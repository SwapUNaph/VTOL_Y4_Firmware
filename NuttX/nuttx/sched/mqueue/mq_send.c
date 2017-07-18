/****************************************************************************
 *  sched/mqueue/mq_send.c
 *
 *   Copyright (C) 2007, 2009, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include  <nuttx/config.h>

#include  <sys/types.h>
#include  <mqueue.h>
#include  <errno.h>
#include  <debug.h>

#include  <nuttx/irq.h>
#include  <nuttx/arch.h>

#include  "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  The "msglen" parameter specifies the length of the message
 *   in bytes pointed to by "msg."  This length must not exceed the maximum
 *   message length from the mq_getattr().
 *
 *   If the message queue is not full, mq_send() place the message in the
 *   message queue at the position indicated by the "prio" argument.
 *   Messages with higher priority will be inserted before lower priority
 *   messages.  The value of "prio" must not exceed MQ_PRIO_MAX.
 *
 *   If the specified message queue is full and O_NONBLOCK is not set in the
 *   message queue, then mq_send() will block until space becomes available
 *   to the queue the message.
 *
 *   If the message queue is full and O_NONBLOCK is set, the message is not
 *   queued and ERROR is returned.
 *
 * Parameters:
 *   mqdes - Message queue descriptor
 *   msg - Message to send
 *   msglen - The length of the message in bytes
 *   prio - The priority of the message
 *
 * Return Value:
 *   On success, mq_send() returns 0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set to indicate the error:
 *
 *   EAGAIN   The queue was full and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 * Assumptions/restrictions:
 *
 ****************************************************************************/

int mq_send(mqd_t mqdes, FAR const char *msg, size_t msglen, int prio)
{
  FAR struct mqueue_inode_s  *msgq;
  FAR struct mqueue_msg_s *mqmsg = NULL;
  irqstate_t flags;
  int ret = ERROR;

  /* Verify the input parameters -- setting errno appropriately
   * on any failures to verify.
   */

  if (mq_verifysend(mqdes, msg, msglen, prio) != OK)
    {
      return ERROR;
    }

  /* Get a pointer to the message queue */

  sched_lock();
  msgq = mqdes->msgq;

  /* Allocate a message structure:
   * - Immediately if we are called from an interrupt handler.
   * - Immediately if the message queue is not full, or
   * - After successfully waiting for the message queue to become
   *   non-FULL.  This would fail with EAGAIN, EINTR, or ETIMEOUT.
   */

  flags = enter_critical_section();
  if (up_interrupt_context()      || /* In an interrupt handler */
      msgq->nmsgs < msgq->maxmsgs || /* OR Message queue not full */
      mq_waitsend(mqdes) == OK)      /* OR Successfully waited for mq not full */
    {
      /* Allocate the message */

      leave_critical_section(flags);
      mqmsg = mq_msgalloc();

      /* Check if the message was sucessfully allocated */

      if (mqmsg == NULL)
        {
          /* No... mq_msgalloc() does not set the errno value */

          set_errno(ENOMEM);
        }
    }
  else
    {
      /* We cannot send the message (and didn't even try to allocate it)
       * because:
       * - We are not in an interrupt handler AND
       * - The message queue is full AND
       * - When we tried waiting, the wait was unsuccessful.
       *
       * In this case mq_waitsend() has already set the errno value.
       */

      leave_critical_section(flags);
    }

  /* Check if we were able to get a message structure -- this can fail
   * either because we cannot send the message (and didn't bother trying
   * to allocate it) or because the allocation failed.
   */

  if (mqmsg != NULL)
    {
      /* The allocation was successful (implying that we can also send the
       * message). Perform the message send.
       *
       * NOTE: There is a race condition here: What if a message is added by
       * interrupt related logic so that queue again becomes non-empty.
       * That is handled because mq_dosend() will permit the maxmsgs limit
       * to be exceeded in that case.
       */

      ret = mq_dosend(mqdes, mqmsg, msg, msglen, prio);
    }

  sched_unlock();
  return ret;
}
