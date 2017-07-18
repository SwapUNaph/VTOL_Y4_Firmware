/****************************************************************************
 * drivers/wireless/cc3000.c
 *
 *   Copyright (C) 2013-2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David_s5 <david_s5@nscdg.com>
 *
 * References:
 *   CC30000 from Texas Instruments http://processors.wiki.ti.com/index.php/CC3000
 *
 * See also:
 *     http://processors.wiki.ti.com/index.php/CC3000_Host_Driver_Porting_Guide
 *     http://processors.wiki.ti.com/index.php/CC3000_Host_Programming_Guide
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/time.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/include/cc3000_upif.h>
#include <nuttx/wireless/cc3000/cc3000_common.h>
#include <nuttx/wireless/cc3000/hci.h>
#include "cc3000_socket.h"
#include "cc3000.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef CCASSERT
#define CCASSERT(predicate) _x_CCASSERT_LINE(predicate, __LINE__)
#define _x_CCASSERT_LINE(predicate, line) typedef char constraint_violated_on_line_##line[2*((predicate)!=0)-1];
#endif

CCASSERT(sizeof(cc3000_buffer_desc) <= CONFIG_MQ_MAXMSGSIZE);

#ifndef CONFIG_CC3000_WORKER_THREAD_PRIORITY
#  define CONFIG_CC3000_WORKER_THREAD_PRIORITY (SCHED_PRIORITY_MAX)
#endif

#ifndef CONFIG_CC3000_WORKER_STACKSIZE
#  define CONFIG_CC3000_WORKER_STACKSIZE 240
#endif

#ifndef CONFIG_CC3000_SELECT_THREAD_PRIORITY
#  define CONFIG_CC3000_SELECT_THREAD_PRIORITY (SCHED_PRIORITY_DEFAULT-1)
#endif

#ifndef CONFIG_CC3000_SELECT_STACKSIZE
#  define CONFIG_CC3000_SELECT_STACKSIZE 368
#endif

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif
#define NUMBER_OF_MSGS 1

#define FREE_SLOT -1
#define CLOSE_SLOT -2

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_CC3000_PROBES)
#  define CC3000_GUARD (0xc35aa53c)
#  define INIT_GUARD(p) p->guard = CC3000_GUARD
#  define CHECK_GUARD(p) DEBUGASSERT(p->guard == CC3000_GUARD)
#  define PROBE(pin,state)  priv->config->probe(priv->config,pin, state)
#else
#  define INIT_GUARD(p)
#  define CHECK_GUARD(p)
#  define PROBE(pin,state)
#endif

#define waiterr(x,...) //  _err
#define waitinfo(x,...) // _info

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

static void cc3000_lock_and_select(FAR struct spi_dev_s *spi);
static void cc3000_deselect_and_unlock(FAR struct spi_dev_s *spi);

/* Interrupts and data sampling */

static void cc3000_notify(FAR struct cc3000_dev_s *priv);
static void *cc3000_worker(FAR void *arg);
static int cc3000_interrupt(int irq, FAR void *context);

/* Character driver methods */

static int cc3000_open(FAR struct file *filep);
static int cc3000_close(FAR struct file *filep);
static ssize_t cc3000_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static ssize_t cc3000_write(FAR struct file *filep,
                            FAR const char *buffer, size_t len);
static int cc3000_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int cc3000_poll(FAR struct file *filep, struct pollfd *fds,
                       bool setup);
#endif

static int cc3000_wait_data(FAR struct cc3000_dev_s *priv, int sockfd);
static int cc3000_accept_socket(FAR struct cc3000_dev_s *priv, int sd,
                                FAR struct sockaddr *addr,
                                socklen_t *addrlen);
static int cc3000_add_socket(FAR struct cc3000_dev_s *priv, int sd);
static int cc3000_remove_socket(FAR struct cc3000_dev_s *priv, int sd);
static int cc3000_remote_closed_socket(FAR struct cc3000_dev_s *priv, int sd);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations cc3000_fops =
{
  cc3000_open,      /* open */
  cc3000_close,     /* close */
  cc3000_read,      /* read */
  cc3000_write,     /* write */
  0,                /* seek */
  cc3000_ioctl,     /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  cc3000_poll       /* poll */
#endif
};

/* If only a single CC3000 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_CC3000_MULTIPLE
static struct cc3000_dev_s g_cc3000;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct cc3000_dev_s *g_cc3000list;
#endif

uint8_t spi_readCommand[] = READ_COMMAND;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: cc3000_devtake() and cc3000_devgive()
 *
 * Description:
 *   Used to get exclusive access to a CC3000 driver.
 *
 ****************************************************************************/

static inline void cc3000_devtake(FAR struct cc3000_dev_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->devsem) < 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(errno == EINTR);
    }
}

static inline void cc3000_devgive(FAR struct cc3000_dev_s *priv)
{
  (void)sem_post(&priv->devsem);
}

/****************************************************************************
 * Function: cc3000_configspi
 *
 * Description:
 *   Configure the SPI for use with the CC3000.  This function should be
 *   called to configure the SPI
 *   bus.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void cc3000_configspi(FAR struct spi_dev_s *spi)
{
  ninfo("Mode: %d Bits: 8 Frequency: %d\n",
       CONFIG_CC3000_SPI_MODE, CONFIG_CC3000_SPI_FREQUENCY);

  SPI_SETMODE(spi, CONFIG_CC3000_SPI_MODE);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, CONFIG_CC3000_SPI_FREQUENCY);
}

/****************************************************************************
 * Function: cc3000_lock
 *
 * Description:
 *   Lock the SPI bus and re-configure as necessary.  This function must be
 *   to assure: (1) exclusive access to the SPI bus, and (2) to assure that
 *   the shared bus is properly configured for the cc3000 module.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void cc3000_lock_and_select(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus so that we have exclusive access */

  (void)SPI_LOCK(spi, true);

  /* We have the lock.  Now make sure that the SPI bus is configured for the
   * CC3000 (it might have gotten configured for a different device while
   * unlocked)
   */

  cc3000_configspi(spi);
  SPI_SELECT(spi, SPIDEV_WIRELESS, true);
}

/****************************************************************************
 * Function: cc3000_unlock
 *
 * Description:
 *   Un-lock the SPI bus after each transfer, possibly losing the current
 *   configuration if we are sharing the SPI bus with other devices.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void cc3000_deselect_and_unlock(FAR struct spi_dev_s *spi)
{
   /* De select */

  SPI_SELECT(spi, SPIDEV_WIRELESS, false);

  /* Relinquish the SPI bus. */

  (void)SPI_LOCK(spi, false);
}

/****************************************************************************
 * Function: cc3000_wait
 *
 * Description:
 *  Helper function to wait on the semaphore signaled by the
 *
 * Parameters:
 *   priv - Reference to the CC3000 driver structure
 *   priv -
 *
 * Returned Value:
 *   0 - Semaphore signaled and devsem retaken
 *  < 0 - Some Error occurred
 * Assumptions:
 *   Own the devsem on entry
 *
 ****************************************************************************/

static int cc3000_wait(FAR struct cc3000_dev_s *priv, sem_t *psem)
{
  int ret;

  /* Give up */

  sched_lock();
  cc3000_devgive(priv);

  /* Wait on first psem to become signaled */

  ret = sem_wait(psem);
  if (ret < 0)
    {
      return -errno;
    }

  /* Then retake the mutual exclusion semaphore */

  cc3000_devtake(priv);
  sched_unlock();
  return OK;
}

/****************************************************************************
 * Function: cc3000_wait_irq
 *
 * Description:
 *  Helper function to wait on the irqsem signaled by the interrupt
 *
 * Parameters:
 *   priv - Reference to the CC3000 driver structure
 *
 * Returned Value:
 *   0 - Semaphore signaled and devsem retaken
 *  < 0 - Some Error occurred
 * Assumptions:
 *   Own the devsem on entry
 *
 ****************************************************************************/

static inline int cc3000_wait_irq(FAR struct cc3000_dev_s *priv)
{
  return cc3000_wait(priv, &priv->irqsem);
}

/****************************************************************************
 * Function: cc3000_wait_ready
 *
 * Description:
 *  Helper function to wait on the readysem signaled by the interrupt
 *
 * Parameters:
 *   priv - Reference to the CC3000 driver structure
 *
 * Returned Value:
 *   0 - Semaphore signaled and devsem retaken
 *  < 0 - Some Error occurred
 * Assumptions:
 *   Own the devsem on entry
 *
 ****************************************************************************/

static inline int cc3000_wait_ready(FAR struct cc3000_dev_s *priv)
{
  return cc3000_wait(priv, &priv->readysem);
}

/****************************************************************************
 * Name: cc3000_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void cc3000_pollnotify(FAR struct cc3000_dev_s *priv, uint32_t type)
{
  int i;

  for (i = 0; i < CONFIG_CC3000_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= type;
          ninfo("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
}
#endif

/****************************************************************************
 * Name: cc3000_notify
 ****************************************************************************/

static void cc3000_notify(FAR struct cc3000_dev_s *priv)
{
  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the CC3000
       * is no longer available.
       */

      sem_post(&priv->waitsem);
    }

  /* If there are threads waiting on poll() for CC3000 data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  cc3000_pollnotify(priv, POLLIN);
#endif
}

/****************************************************************************
 * Name: cc3000_worker
 ****************************************************************************/

static void * select_thread_func(FAR void *arg)
{
  FAR struct cc3000_dev_s *priv = (FAR struct cc3000_dev_s *)arg;
  struct timeval timeout;
  TICC3000fd_set readsds;
  TICC3000fd_set exceptsds;
  int ret = 0;
  int maxFD = 0;
  int s = 0;

  memset(&timeout, 0, sizeof(struct timeval));
  timeout.tv_sec  = 0;
  timeout.tv_usec = (500000);  /* 500 msecs */

  while (1)
    {
      sem_wait(&priv->selectsem);

      CHECK_GUARD(priv);

      /* Increase the count back by one to be decreased by the original caller */

      sem_post(&priv->selectsem);

      maxFD = -1;
      CC3000_FD_ZERO(&readsds);
      CC3000_FD_ZERO(&exceptsds);

      /* Ping correct socket descriptor param for select */

      for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
        {
          if (priv->sockets[s].sd != FREE_SLOT)
            {
              if (priv->sockets[s].sd == CLOSE_SLOT)
                {
                  priv->sockets[s].sd = FREE_SLOT;
                  waitinfo("Close\n");
                  int count;
                  do
                    {
                      sem_getvalue(&priv->sockets[s].semwait, &count);
                      if (count < 0)
                        {
                          /* Release the waiting threads */

                          waitinfo("Closed Signaled %d\n", count);
                          sem_post(&priv->sockets[s].semwait);
                        }
                    }
                  while (count < 0);

                  continue;
                }

              CC3000_FD_SET(priv->sockets[s].sd, &readsds);
              CC3000_FD_SET(priv->sockets[s].sd, &exceptsds);
              if (maxFD <= priv->sockets[s].sd)
                {
                  maxFD = priv->sockets[s].sd + 1;
                }
            }
        }

      if (maxFD < 0)
        {
          /* Handled only socket close. */

          continue;
        }

      /* Polling instead of blocking here to process "accept" below */

      ret = cc3000_select(maxFD, (fd_set *) &readsds, NULL,
                         (fd_set *) &exceptsds, &timeout);

      if (priv->selecttid == -1)
        {
          /* driver close will terminate the thread and by that all sync
           * objects owned by it will be released
           */

          return OK;
        }

      for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
        {
          if ((priv->sockets[s].sd != FREE_SLOT ||
               priv->sockets[s].sd != CLOSE_SLOT) &&                         /* Check that the socket is valid */
               priv->sockets[s].sd != priv->accepting_socket.acc.sd)         /* Verify this is not an accept socket */
            {
              if (ret > 0 && CC3000_FD_ISSET(priv->sockets[s].sd, &readsds)) /* and has pending data */
                {
                  waitinfo("Signaled %d\n", priv->sockets[s].sd);
                  sem_post(&priv->sockets[s].semwait);                       /* release the waiting thread */
                }
              else if (ret > 0 && CC3000_FD_ISSET(priv->sockets[s].sd, &exceptsds)) /* or has pending exception */
                {
                  waitinfo("Signaled %d (exception)\n", priv->sockets[s].sd);
                  sem_post(&priv->sockets[s].semwait);                       /* release the waiting thread */
                }
              else if (priv->sockets[s].received_closed_event)               /* or remote has closed connection and we have now read all of HW buffer. */
                {
                  waitinfo("Signaled %d (closed & empty)\n", priv->sockets[s].sd);
                  priv->sockets[s].emptied_and_remotely_closed = true;
                  sem_post(&priv->sockets[s].semwait);                       /* release the waiting thread */
                }
            }
        }

      if (priv->accepting_socket.acc.sd != FREE_SLOT)                        /* If accept polling in needed */
        {
          if (priv->accepting_socket.acc.sd == CLOSE_SLOT)
            {
              ret = CC3000_SOC_ERROR;
            }
            else
            {
              ret = cc3000_do_accept(priv->accepting_socket.acc.sd,              /* Send the select command on non blocking */
                                     &priv->accepting_socket.addr,               /* Set up in ioctl */
                                     &priv->accepting_socket.addrlen);
            }

          if (ret != CC3000_SOC_IN_PROGRESS)                                 /* Not waiting => error or accepted */
            {
              priv->accepting_socket.acc.sd = FREE_SLOT;
              priv->accepting_socket.acc.status = ret;
              sem_post(&priv->accepting_socket.acc.semwait);                 /* Release the waiting thread */
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cc3000_worker
 ****************************************************************************/

static void * cc3000_worker(FAR void *arg)
{
  FAR struct cc3000_dev_s *priv = (FAR struct cc3000_dev_s *)arg;
  int ret;

  ASSERT(priv != NULL && priv->config != NULL);

  /* We have started, release our creator */

  sem_post(&priv->readysem);
  while (1)
    {
      PROBE(0, 1);
      CHECK_GUARD(priv);
      cc3000_devtake(priv);

      /* Done ? */

      if ((cc3000_wait_irq(priv) != -EINTR) && (priv->workertid != -1))
        {
          PROBE(0, 0);
          ninfo("State%d\n", priv->state);
          switch (priv->state)
            {
            case eSPI_STATE_POWERUP:
              /* Signal the device has interrupted after power up */

              priv->state = eSPI_STATE_INITIALIZED;
              sem_post(&priv->readysem);
              break;

            case eSPI_STATE_WRITE_WAIT_IRQ:
              /* Signal the device has interrupted after Chip Select During a write operation */

              priv->state = eSPI_STATE_WRITE_PROCEED;
              sem_post(&priv->readysem);
              break;

            case eSPI_STATE_WRITE_DONE:  /* IRQ post a write => Solicited */
            case eSPI_STATE_IDLE: /* IRQ when Idel => cc3000 has data for the hosts Unsolicited */
              {
                uint16_t data_to_recv;
                priv->state = eSPI_STATE_READ_IRQ;

                /* Issue the read command */

                cc3000_lock_and_select(priv->spi); /* Assert CS */
                priv->state = eSPI_STATE_READ_PROCEED;
                SPI_EXCHANGE(priv->spi, spi_readCommand, priv->rx_buffer.pbuffer,
                             ARRAY_SIZE(spi_readCommand));

               /* Extract Length bytes from Rx Buffer.  Here we need to convert
                * unaligned data in network order (big endian, MS byte first) to
                * host order.  We cannot use ntohs here because the data is not
                * aligned.
                */

               data_to_recv = (uint16_t)priv->rx_buffer.pbuffer[READ_OFFSET_TO_LENGTH] << 8 |
                              (uint16_t)priv->rx_buffer.pbuffer[READ_OFFSET_TO_LENGTH + 1];

               if (data_to_recv)
                  {
                    /* We will read ARRAY_SIZE(spi_readCommand) + data_to_recv. is it odd? */

                    if ((data_to_recv +  ARRAY_SIZE(spi_readCommand)) & 1)
                      {
                        /* Odd so make it even */

                        data_to_recv++;
                      }

                    /* Read the whole payload in at the beginning of the buffer
                     * Will it fit?
                     */

                    if (data_to_recv >= priv->rx_buffer_max_len)
                      {
                        ninfo("data_to_recv %d", data_to_recv);
                      }

                    DEBUGASSERT(data_to_recv < priv->rx_buffer_max_len);
                    SPI_RECVBLOCK(priv->spi, priv->rx_buffer.pbuffer, data_to_recv);
                  }

                cc3000_deselect_and_unlock(priv->spi); /* De assert CS */

                /* Disable more messages as the wl code will resume via CC3000IOC_COMPLETE */

                if (data_to_recv)
                  {
                    int count;

                    priv->state = eSPI_STATE_READ_READY;
                    priv->rx_buffer.len = data_to_recv;

                    ret = mq_send(priv->queue, (FAR const char *)&priv->rx_buffer,
                                  sizeof(priv->rx_buffer), 1);
                    DEBUGASSERT(ret >= 0);
                    UNUSED(ret);

                    /* Notify any waiters that new CC3000 data is available */

                    cc3000_notify(priv);

                    /* Give up driver */

                    cc3000_devgive(priv);

                    ninfo("Wait On Completion\n");
                    sem_wait(priv->wrkwaitsem);
                    ninfo("Completed S:%d irq :%d\n",
                          priv->state, priv->config->irq_read(priv->config));

                    sem_getvalue(&priv->irqsem, &count);
                    if (priv->config->irq_read(priv->config) && count == 0)
                      {
                        sem_post(&priv->irqsem);
                      }

                    if (priv->state == eSPI_STATE_READ_READY)
                      {
                         priv->state = eSPI_STATE_IDLE;
                      }

                    continue;
                  }
              }
              break;

            default:
              ninfo("default: State%d\n", priv->state);
              break;
            }
        }

      cc3000_devgive(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: cc3000_interrupt
 ****************************************************************************/

static int cc3000_interrupt(int irq, FAR void *context)
{
  FAR struct cc3000_dev_s    *priv;

  /* Which CC3000 device caused the interrupt? */

#ifndef CONFIG_CC3000_MULTIPLE
  priv = &g_cc3000;
#else
  for (priv = g_cc3000list;
       priv && priv->configs->irq != irq;
       priv = priv->flink);

  ASSERT(priv != NULL);
#endif

  /* Run the worker thread */

  PROBE(1, 0);
  sem_post(&priv->irqsem);
  PROBE(1, 1);

  /* Clear any pending interrupts and return success */

  priv->config->irq_clear(priv->config);
  return OK;
}

/****************************************************************************
 * Name: cc3000_open
 ****************************************************************************/

static int cc3000_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  struct mq_attr attr;
  pthread_attr_t tattr;
  pthread_t threadid;
  struct sched_param param;
  char queuename[QUEUE_NAMELEN];
  FAR struct cc3000_dev_s *priv;
  uint8_t tmp;
  int ret;
#ifdef CONFIG_CC3000_MT
  int s;
#endif

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct cc3000_dev_s *)inode->i_private;

  CHECK_GUARD(priv);

  ninfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the driver data structure */

  cc3000_devtake(priv);

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto out_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  if (tmp == 1)
    {
      /* Initialize semaphores */

      sem_init(&priv->waitsem, 0, 0);  /* Initialize event wait semaphore */
      sem_init(&priv->irqsem, 0, 0);   /* Initialize IRQ Ready semaphore */
      sem_init(&priv->readysem, 0, 0); /* Initialize Device Ready semaphore */

      /* These semaphores are all used for signaling and, hence, should
       * not have priority inheritance enabled.
       */

      sem_setprotocol(&priv->waitsem, SEM_PRIO_NONE);
      sem_setprotocol(&priv->irqsem, SEM_PRIO_NONE);
      sem_setprotocol(&priv->readysem, SEM_PRIO_NONE);

#ifdef CONFIG_CC3000_MT
      priv->accepting_socket.acc.sd = FREE_SLOT;
      sem_init(&priv->accepting_socket.acc.semwait, 0, 0);
      sem_setprotocol(&priv->accepting_socket.acc.semwait, SEM_PRIO_NONE);

      for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
        {
          priv->sockets[s].sd = FREE_SLOT;
          priv->sockets[s].received_closed_event = false;
          priv->sockets[s].emptied_and_remotely_closed = false;

          sem_init(&priv->sockets[s].semwait, 0, 0);
          sem_setprotocol(&priv->sockets[s].semwait, SEM_PRIO_NONE);
        }
#endif

      /* Ensure the power is off  so we get the falling edge of IRQ */

      priv->config->power_enable(priv->config, false);

      attr.mq_maxmsg  = NUMBER_OF_MSGS;
      attr.mq_msgsize = sizeof(cc3000_buffer_desc);
      attr.mq_flags   = 0;

      /* Set the flags for the open of the queue.
       * Make it a blocking open on the queue, meaning it will block if
       * this process tries to send to the queue and the queue is full.
       *
       *   O_CREAT - the queue will get created if it does not already exist.
       *   O_WRONLY - we are only planning to write to the queue.
       *
       * Open the queue, and create it if the receiving process hasn't
       * already created it.
       */

      snprintf(queuename, QUEUE_NAMELEN, QUEUE_FORMAT, priv->minor);
      priv->queue = mq_open(queuename, O_WRONLY | O_CREAT, 0666, &attr);
      if (priv->queue < 0)
        {
          ret = -errno;
          goto errout_sem_destroy;
        }

      pthread_attr_init(&tattr);
      tattr.stacksize = CONFIG_CC3000_WORKER_STACKSIZE;
      param.sched_priority = CONFIG_CC3000_WORKER_THREAD_PRIORITY;
      pthread_attr_setschedparam(&tattr, &param);

      ret = pthread_create(&priv->workertid, &tattr, cc3000_worker,
                           (pthread_addr_t)priv);
      if (ret != 0)
        {
          ret = -ret; /* pthread_create does not modify errno. */
          goto errout_mq_close;
        }

      pthread_attr_init(&tattr);
      tattr.stacksize = CONFIG_CC3000_SELECT_STACKSIZE;
      param.sched_priority = CONFIG_CC3000_SELECT_THREAD_PRIORITY;
      pthread_attr_setschedparam(&tattr, &param);

      sem_init(&priv->selectsem, 0, 0);
      sem_setprotocol(&priv->selectsem, SEM_PRIO_NONE);

      ret = pthread_create(&priv->selecttid, &tattr, select_thread_func,
                           (pthread_addr_t)priv);
      if (ret != 0)
        {
          ret = -ret; /* pthread_create does not modify errno. */
          goto errout_worker_cancel;
        }

      /* Do late allocation with hopes of realloc not fragmenting */

      priv->rx_buffer.pbuffer =  kmm_malloc(priv->rx_buffer_max_len);
      DEBUGASSERT(priv->rx_buffer.pbuffer);
      if (!priv->rx_buffer.pbuffer)
        {
          ret = -errno;
          goto errout_select_cancel;
        }

      priv->state = eSPI_STATE_POWERUP;
      priv->config->irq_clear(priv->config);

      /* Bring the device Online A) on http://processors.wiki.ti.com/index.php/File:CC3000_Master_SPI_Write_Sequence_After_Power_Up.png */

      priv->config->irq_enable(priv->config, true);

      /* Wait on child thread */

      cc3000_wait_ready(priv);
      priv->config->power_enable(priv->config, true);
    }

  /* Save the new open count on success */

  priv->crefs = tmp;
  ret = OK;

  goto out_with_sem;

errout_select_cancel:
  threadid = priv->selecttid;
  priv->selecttid = -1;
  pthread_cancel(threadid);
  pthread_join(threadid, NULL);
  sem_destroy(&priv->selectsem);

errout_worker_cancel:
  threadid = priv->workertid;
  priv->workertid = -1;
  pthread_cancel(threadid);
  pthread_join(threadid, NULL);

errout_mq_close:
  mq_close(priv->queue);
  priv->queue = 0;

errout_sem_destroy:
#ifdef CONFIG_CC3000_MT
  sem_destroy(&priv->accepting_socket.acc.semwait);

  for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
    {
      priv->sockets[s].sd = FREE_SLOT;
      sem_destroy(&priv->sockets[s].semwait);
    }
#endif

  sem_destroy(&priv->waitsem);
  sem_destroy(&priv->irqsem);
  sem_destroy(&priv->readysem);

out_with_sem:
  cc3000_devgive(priv);
  return ret;
}

/****************************************************************************
 * Name: cc3000_close
 ****************************************************************************/

static int cc3000_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cc3000_dev_s *priv;
#ifdef CONFIG_CC3000_MT
  int s;
#endif

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct cc3000_dev_s *)inode->i_private;

  CHECK_GUARD(priv);

  ninfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the driver data structure */

  cc3000_devtake(priv);

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  int tmp = priv->crefs;
  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  if (tmp == 1)
    {
      pthread_t id = priv->selecttid;
      priv->selecttid = -1;
      pthread_cancel(id);
      pthread_join(id, NULL);
      sem_destroy(&priv->selectsem);

      priv->config->irq_enable(priv->config, false);
      priv->config->irq_clear(priv->config);
      priv->config->power_enable(priv->config, false);

      id = priv->workertid;
      priv->workertid = -1;
      pthread_cancel(id);
      pthread_join(id, NULL);

      mq_close(priv->queue);
      priv->queue = 0;

      kmm_free(priv->rx_buffer.pbuffer);
      priv->rx_buffer.pbuffer = 0;

#ifdef CONFIG_CC3000_MT
      sem_destroy(&priv->accepting_socket.acc.semwait);

      for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
        {
          priv->sockets[s].sd = FREE_SLOT;
          sem_destroy(&priv->sockets[s].semwait);
        }
#endif

      sem_destroy(&priv->waitsem);
      sem_destroy(&priv->irqsem);
      sem_destroy(&priv->readysem);
    }

  cc3000_devgive(priv);
  return OK;
}

/****************************************************************************
 * Name: cc3000_read
 ****************************************************************************/

static ssize_t cc3000_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct cc3000_dev_s *priv;
  int ret;
  ssize_t nread;

  ninfo("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  CHECK_GUARD(priv);

  /* Get exclusive access to the driver data structure */

  cc3000_devtake(priv);

  /* Verify that the caller has provided a buffer large enough to receive
   * the maximum data.
   */

  if (len < priv->rx_buffer_max_len)
    {
      nerr("ERROR: Unsupported read size: %d\n", len);
      nread = -ENOSYS;
      goto errout_with_sem;
    }


  for (nread = priv->rx_buffer.len; nread == 0; nread = priv->rx_buffer.len)
    {
      if (nread > 0)
        {
          /* Yes.. break out to return what we have. */

          break;
        }

      /* data is not available now.  We would have to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      ninfo("CC3000 data is not available\n");
      if (filep->f_oflags & O_NONBLOCK)
        {
          nread = -EAGAIN;
          break;
        }

      /* Otherwise, wait for something to be written to the
       * buffer. Increment the number of waiters so that the notify
       * will know that it needs to post the semaphore to wake us up.
       */

      sched_lock();
      priv->nwaiters++;
      cc3000_devgive(priv);

      /* We may now be pre-empted!  But that should be okay because we
       * have already incremented nwaiters.  Pre-emptions is disabled
       * but will be re-enabled while we are waiting.
       */

      ninfo("Waiting..\n");
      ret = sem_wait(&priv->waitsem);
      priv->nwaiters--;
      sched_unlock();

      /* Did we successfully get the waitsem? */

      if (ret >= 0)
        {
          /* Yes... then retake the mutual exclusion semaphore */

          cc3000_devtake(priv);
        }

      /* Was the semaphore wait successful? Did we successful re-take the
       * mutual exclusion semaphore?
       */

      if (ret < 0)
        {
          /* No.. One of the two sem_wait's failed. */

          int errval = errno;

          /* Were we awakened by a signal?  Did we read anything before
           * we received the signal?
           */

          if (errval != EINTR || nread  >= 0)
            {
              /* Yes.. return the error. */

              nread = -errval;
            }

          /* Break out to return what we have.  Note, we can't exactly
           * "break" out because whichever error occurred, we do not hold
           * the exclusion semaphore.
           */

          goto errout_without_sem;
        }
    }

  if (nread > 0)
    {
      memcpy(buffer, priv->rx_buffer.pbuffer, priv->rx_buffer.len);
      priv->rx_buffer.len = 0;
     }

errout_with_sem:
  cc3000_devgive(priv);

errout_without_sem:
  ninfo("Returning: %d\n", nread);
#ifndef CONFIG_DISABLE_POLL
  if (nread > 0)
    {
      cc3000_pollnotify(priv, POLLOUT);
    }
#endif

  return nread;
}

/****************************************************************************
 * Name:cc3000_write
 *
 * Bit of non standard buffer management ahead
 * The buffer is memory allocated in the user space with space for the spi
 * header
 *
 ****************************************************************************/

static ssize_t cc3000_write(FAR struct file *filep, FAR const char *usrbuffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct cc3000_dev_s *priv;
  FAR char *buffer = (FAR char *) usrbuffer;
  ssize_t nwritten = 0;
  int ret;

  /* Set the padding if count(buffer) is even ( as it will be come odd with header) */

  size_t tx_len = (len & 1) ? len : len +1;

  ninfo("buffer:%p len:%d tx_len:%d\n", buffer, len, tx_len);

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  CHECK_GUARD(priv);

  /* Get exclusive access to the driver data structure */

  cc3000_devtake(priv);

  /* Figure out the total length of the packet in order to figure out if there is padding or not */

  buffer[0] = WRITE;
  buffer[1] = HI(tx_len);
  buffer[2] = LO(tx_len);
  buffer[3] = 0;
  buffer[4] = 0;
  tx_len += SPI_HEADER_SIZE;

  /* The first write transaction to occur after release of the shutdown has
   * slightly different timing than the others.  The normal Master SPI
   * write sequence is nCS low, followed by IRQ low (CC3000 host),
   * indicating that the CC3000 core device is ready to accept data.
   * However, after power up the sequence is slightly different, as shown
   * in the following Figure:
   *
   *   http://processors.wiki.ti.com/index.php/File:CC3000_Master_SPI_Write_Sequence_After_Power_Up.png
   *
   * The following is a sequence of operations:
   *  - The master detects the IRQ line low: in this case the detection of
   *    IRQ low does not indicate the intention of the CC3000 device to
   *    communicate with the master but rather CC3000 readiness after power
   *    up.
   *  - The master asserts nCS.
   *  - The master introduces a delay of at least 50 μs before starting
   *    actual transmission of data.
   *  - The master transmits the first 4 bytes of the SPI header.
   *  - The master introduces a delay of at least an additional 50 μs.
   *  - The master transmits the rest of the packet.
   */

  if (priv->state == eSPI_STATE_POWERUP)
    {
      ret = cc3000_wait_ready(priv);
      if (ret < 0)
        {
          nwritten = ret;
          goto errout_without_sem;
        }
    }

  if (priv->state  == eSPI_STATE_INITIALIZED)
    {
      cc3000_lock_and_select(priv->spi); /* Assert CS */
      usleep(55);
      SPI_SNDBLOCK(priv->spi, buffer, 4);
      usleep(55);
      SPI_SNDBLOCK(priv->spi, buffer+4, tx_len-4);
    }
  else
    {
      ninfo("Assert CS\n");
      priv->state  = eSPI_STATE_WRITE_WAIT_IRQ;
      cc3000_lock_and_select(priv->spi); /* Assert CS */
      ninfo("Wait on IRQ Active\n");
      ret = cc3000_wait_ready(priv);
      ninfo("IRQ Signaled\n");
      if (ret < 0)
        {
          /* This should only happen if the wait was canceled by an signal */

          cc3000_deselect_and_unlock(priv->spi);
          ninfo("sem_wait: %d\n", errno);
          DEBUGASSERT(errno == EINTR);
          nwritten = ret;
          goto errout_without_sem;
        }

      SPI_SNDBLOCK(priv->spi, buffer, tx_len);
    }

  priv->state  = eSPI_STATE_WRITE_DONE;
  ninfo("Deassert CS S:eSPI_STATE_WRITE_DONE\n");
  cc3000_deselect_and_unlock(priv->spi);
  nwritten = tx_len;
  cc3000_devgive(priv);

errout_without_sem:
  ninfo("Returning: %d\n", ret);
  return nwritten;
}

/****************************************************************************
 * Name:cc3000_ioctl
 ****************************************************************************/

static int cc3000_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct cc3000_dev_s *priv;
  int ret;

  ninfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct cc3000_dev_s *)inode->i_private;

  CHECK_GUARD(priv);

  /* Get exclusive access to the driver data structure */

  cc3000_devtake(priv);

  /* Process the IOCTL by command */

  ret = OK;
  switch (cmd)
    {
      case CC3000IOC_GETQUESEMID:
        {
          FAR int *pminor = (FAR int *)(arg);
          DEBUGASSERT(pminor != NULL);
          *pminor = priv->minor;
          break;
        }

      case CC3000IOC_ADDSOCKET:
        {
          FAR int *pfd = (FAR int *)(arg);
          DEBUGASSERT(pfd != NULL);
          *pfd = cc3000_add_socket(priv, *pfd);
          break;
        }

      case CC3000IOC_REMOVESOCKET:
        {
          FAR int *pfd = (FAR int *)(arg);
          DEBUGASSERT(pfd != NULL);
          *pfd = cc3000_remove_socket(priv, *pfd);
          break;
        }

      case CC3000IOC_REMOTECLOSEDSOCKET:
        {
          FAR int *pfd = (FAR int *)(arg);
          DEBUGASSERT(pfd != NULL);
          *pfd = cc3000_remote_closed_socket(priv, *pfd);
          break;
        }

      case CC3000IOC_SELECTDATA:
        {
          FAR int *pfd = (FAR int *)(arg);
          DEBUGASSERT(pfd != NULL);
          *pfd = cc3000_wait_data(priv, *pfd);
          break;
        }

      case CC3000IOC_SELECTACCEPT:
        {
          FAR cc3000_acceptcfg *pcfg = (FAR cc3000_acceptcfg *)(arg);
          DEBUGASSERT(pcfg != NULL);
          pcfg->sockfd = cc3000_accept_socket(priv, pcfg->sockfd, pcfg->addr, pcfg->addrlen);
          break;
        }

      case CC3000IOC_SETRX_SIZE:
        {
          irqstate_t flags;
          FAR int *psize = (FAR int *)(arg);
          int rv;

          DEBUGASSERT(psize != NULL);
          rv = priv->rx_buffer_max_len;
          flags = enter_critical_section();
          priv->rx_buffer_max_len = *psize;
          priv->rx_buffer.pbuffer = kmm_realloc(priv->rx_buffer.pbuffer, *psize);
          leave_critical_section(flags);
          DEBUGASSERT(priv->rx_buffer.pbuffer);
          *psize = rv;
          break;
        }

      default:
        ret = -ENOTTY;
        break;
    }

  cc3000_devgive(priv);
  return ret;
}

/****************************************************************************
 * Name: cc3000_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int cc3000_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct cc3000_dev_s *priv;
  int ret = OK;
  int i;

  ninfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct cc3000_dev_s *)inode->i_private;

  CHECK_GUARD(priv);

  /* Are we setting up the poll?  Or tearing it down? */

  cc3000_devtake(priv);

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_CC3000_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_CC3000_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->rx_buffer.len)
        {
          cc3000_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  cc3000_devgive(priv);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc3000_register
 *
 * Description:
 *   Configure the CC3000 to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int cc3000_register(FAR struct spi_dev_s *spi,
                      FAR struct cc3000_config_s *config, int minor)
{
  FAR struct cc3000_dev_s *priv;
  char drvname[DEV_NAMELEN];
  char semname[SEM_NAMELEN];
#ifdef CONFIG_CC3000_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  ninfo("spi: %p minor: %d\n", spi, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(spi != NULL && config != NULL && minor >= 0 && minor < 100);

  /* Create and initialize a CC3000 device driver instance */

#ifndef CONFIG_CC3000_MULTIPLE
  priv = &g_cc3000;
#else
  priv = (FAR struct cc3000_dev_s *)kmm_malloc(sizeof(struct cc3000_dev_s));
  if (!priv)
    {
      nerr("ERROR: kmm_malloc(%d) failed\n", sizeof(struct cc3000_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the CC3000 device driver instance */

  memset(priv, 0, sizeof(struct cc3000_dev_s));
  INIT_GUARD(priv);
  priv->minor  = minor;              /* Save the minor number */
  priv->spi    = spi;                /* Save the SPI device handle */
  priv->config = config;             /* Save the board configuration */

  priv->rx_buffer_max_len = config->max_rx_size;

  sem_init(&priv->devsem,  0, 1);    /* Initialize device structure semaphore */

  (void)snprintf(semname, SEM_NAMELEN, SEM_FORMAT, minor);
  priv->wrkwaitsem = sem_open(semname, O_CREAT, 0, 0); /* Initialize  Worker Wait semaphore */

#ifdef CONFIG_CC3000_MT
  pthread_mutex_init(&g_cc3000_mut, NULL);
#endif

  /* Make sure that interrupts are disabled */

  config->irq_clear(config);
  config->irq_enable(config, false);

  /* Attach the interrupt handler */

  ret = config->irq_attach(config, cc3000_interrupt);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Register the device as an input device */

  (void)snprintf(drvname, DEV_NAMELEN, DEV_FORMAT, minor);
  ninfo("Registering %s\n", drvname);

  ret = register_driver(drvname, &cc3000_fops, 0666, priv);
  if (ret < 0)
    {
      nerr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* If multiple CC3000 devices are supported, then we will need to add
   * this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the recieved IRQ number.
   */

#ifdef CONFIG_CC3000_MULTIPLE
  priv->flink    = g_cc3000list;
  g_cc3000list = priv;
  leave_critical_section(flags);
#endif

  /* And return success (?) */

  return OK;

errout_with_priv:
  sem_destroy(&priv->devsem);
  sem_close(priv->wrkwaitsem);
  sem_unlink(semname);

#ifdef CONFIG_CC3000_MT
  pthread_mutex_destroy(&g_cc3000_mut);
#endif

#ifdef CONFIG_CC3000_MULTIPLE
  kmm_free(priv);
#endif
  return ret;
}

/****************************************************************************
 * Name: cc3000_wait_data
 *
 * Description:
 *   Adds this socket for monitoring for the data available
 *
 * Input Parameters:
 *   priv   - The device cc3000_dev_s instance
 *   sockfd   cc3000 socket handle
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found or shut down occured.
 *
 ****************************************************************************/

static int cc3000_wait_data(FAR struct cc3000_dev_s *priv, int sockfd)
{
  int s;

  for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
    {
      if (priv->sockets[s].sd == sockfd)
        {
          sched_lock();
          cc3000_devgive(priv);
          sem_post(&priv->selectsem);           /* Wake select thread if need be */
          sem_wait(&priv->sockets[s].semwait);  /* Wait caller on select to finish */
          sem_wait(&priv->selectsem);           /* Sleep select thread */
          cc3000_devtake(priv);
          sched_unlock();

          if (!priv->sockets[s].sd == sockfd)
            {
              return -1;
            }

          if (priv->sockets[s].emptied_and_remotely_closed)
            {
              return -ECONNABORTED;
            }

          return OK;
        }
    }

  return (s >= CONFIG_WL_MAX_SOCKETS || priv->selecttid == -1) ? -1 : OK;
}

/****************************************************************************
 * Name: cc3000_accept_socket
 *
 * Description:
 *   Adds this socket for monitoring for the accept operation
 *
 * Input Parameters:
 *   priv   - The device cc3000_dev_s instance
 *   sockfd - cc3000 socket handle to monitor
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negative value is
 *   returned to indicate an error.
 *
 ****************************************************************************/

static int cc3000_accept_socket(FAR struct cc3000_dev_s *priv, int sd, struct sockaddr *addr,
                         socklen_t *addrlen)
{

  priv->accepting_socket.acc.status = CC3000_SOC_ERROR;
  priv->accepting_socket.acc.sd = sd;

  sched_lock();
  cc3000_devgive(priv);
  sem_post(&priv->selectsem);                    /* Wake select thread if need be */
  sem_wait(&priv->accepting_socket.acc.semwait); /* Wait caller on select to finish */
  sem_wait(&priv->selectsem);                    /* Sleep the Thread */
  cc3000_devtake(priv);
  sched_unlock();

  if (priv->accepting_socket.acc.status != CC3000_SOC_ERROR)
    {
      *addr = priv->accepting_socket.addr;
      *addrlen = priv->accepting_socket.addrlen;
      cc3000_add_socket(priv, priv->accepting_socket.acc.status);
    }

  return priv->accepting_socket.acc.status;
}

/****************************************************************************
 * Name: cc3000_add_socket
 *
 * Description:
 *   Adds a socket to the list for monitoring for long operation
 *
 * Input Parameters:
 *   sd      cc3000 socket handle
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

static int cc3000_add_socket(FAR struct cc3000_dev_s *priv, int sd)
{
  irqstate_t flags;
  int s;

  if (sd < 0)
    {
      return sd;
    }

  flags = enter_critical_section();
  for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
    {
      if (priv->sockets[s].sd == FREE_SLOT)
        {
          priv->sockets[s].received_closed_event = false;
          priv->sockets[s].emptied_and_remotely_closed = false;
          priv->sockets[s].sd = sd;
          break;
        }
    }

  leave_critical_section(flags);
  return s >= CONFIG_WL_MAX_SOCKETS ? -1 : OK;
}

/****************************************************************************
 * Name: cc3000_remove_socket
 *
 * Description:
 *   Removes a socket from the list of monitoring for long operation
 *
 * Input Parameters:
 *   sd      cc3000 socket handle
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

static int cc3000_remove_socket(FAR struct cc3000_dev_s *priv, int sd)
{
  irqstate_t flags;
  int s;
  sem_t *ps = 0;

  if (sd < 0)
    {
      return sd;
    }

  flags = enter_critical_section();
  if (priv->accepting_socket.acc.sd == sd)
    {
      priv->accepting_socket.acc.sd = CLOSE_SLOT;
      ps = &priv->accepting_socket.acc.semwait;
    }

  for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
    {
      if (priv->sockets[s].sd == sd)
        {
          priv->sockets[s].received_closed_event = false;
          priv->sockets[s].emptied_and_remotely_closed = false;
          priv->sockets[s].sd = CLOSE_SLOT;
          ps = &priv->sockets[s].semwait;
          break;
        }
    }

  leave_critical_section(flags);
  if (ps)
    {
      sched_lock();
      cc3000_devgive(priv);
      sem_post(&priv->selectsem);                    /* Wake select thread if need be */
      sem_wait(ps);
      sem_wait(&priv->selectsem);                    /* Sleep the Thread */
      cc3000_devtake(priv);
      sched_unlock();
    }

  return s >= CONFIG_WL_MAX_SOCKETS ? -1 : OK;
}

/****************************************************************************
 * Name: cc3000_remote_closed_socket
 *
 * Description:
 *   Mark socket as closed by remote host
 *
 * Input Parameters:
 *   sd      cc3000 socket handle
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

static int cc3000_remote_closed_socket(FAR struct cc3000_dev_s *priv, int sd)
{
  irqstate_t flags;
  int s;

  if (sd < 0)
    {
      return sd;
    }

  flags = enter_critical_section();
  for (s = 0; s < CONFIG_WL_MAX_SOCKETS; s++)
    {
      if (priv->sockets[s].sd == sd)
        {
          priv->sockets[s].received_closed_event = true;
        }
    }

  leave_critical_section(flags);

  return s >= CONFIG_WL_MAX_SOCKETS ? -1 : OK;
}
