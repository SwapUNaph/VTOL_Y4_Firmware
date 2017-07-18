/****************************************************************************
 * system/composite/composite_main.c
 *
 *   Copyright (C) 2012-2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/composite.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/usbdev_trace.h>

#include "composite.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* All global variables used by this add-on are packed into a structure in
 * order to avoid name collisions.
 */

struct composite_state_s g_composite;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_memory_usage
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_COMPOSITE_DEBUGMM
static void show_memory_usage(struct mallinfo *mmbefore,
                              struct mallinfo *mmafter)
{
  int diff;

  printf("              total       used       free    largest\n");
  printf("Before:%11d%11d%11d%11d\n",
         mmbefore->arena, mmbefore->uordblks, mmbefore->fordblks,
         mmbefore->mxordblk);
  printf("After: %11d%11d%11d%11d\n",
         mmafter->arena, mmafter->uordblks, mmafter->fordblks,
         mmafter->mxordblk);

  diff = mmbefore->uordblks - mmafter->uordblks;
  if (diff < 0)
    {
      printf("Change:%11d allocated\n", -diff);
    }
  else if (diff > 0)
    {
      printf("Change:%11d freed\n", diff);
    }
}
#else
# define show_memory_usage(mm1, mm2)
#endif

/****************************************************************************
 * Name: check_test_memory_usage
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_COMPOSITE_DEBUGMM
static void check_test_memory_usage(FAR const char *msg)
{
  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_composite.mmcurrent = mallinfo();
#else
  (void)mallinfo(&g_composite.mmcurrent);
#endif

  /* Show the change from the previous time */

  printf("\%s:\n", msg);
  show_memory_usage(&g_composite.mmprevious, &g_composite.mmcurrent);

  /* Set up for the next test */

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_composite.mmprevious = g_composite.mmcurrent;
#else
  memcpy(&g_composite.mmprevious, &g_composite.mmcurrent, sizeof(struct mallinfo));
#endif
}
#else
#  define check_test_memory_usage(msg)
#endif

/****************************************************************************
 * Name: check_test_memory_usage
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_COMPOSITE_DEBUGMM
static void final_memory_usage(FAR const char *msg)
{
  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_composite.mmcurrent = mallinfo();
#else
  (void)mallinfo(&g_composite.mmcurrent);
#endif

  /* Show the change from the previous time */

  printf("\n%s:\n", msg);
  show_memory_usage(&g_composite.mmstart, &g_composite.mmcurrent);
}
#else
#  define final_memory_usage(msg)
#endif

/****************************************************************************
 * Name: composite_enumerate
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int composite_enumerate(struct usbtrace_s *trace, void *arg)
{
  switch (trace->event)
    {
    case TRACE_DEVINIT:
      printf("USB controller initialization: %04x\n", trace->value);
      break;

    case TRACE_DEVUNINIT:
      printf("USB controller un-initialization: %04x\n", trace->value);
      break;

    case TRACE_DEVREGISTER:
      printf("usbdev_register(): %04x\n", trace->value);
      break;

    case TRACE_DEVUNREGISTER:
      printf("usbdev_unregister(): %04x\n", trace->value);
      break;

    case TRACE_EPCONFIGURE:
      printf("Endpoint configure(): %04x\n", trace->value);
      break;

    case TRACE_EPDISABLE:
      printf("Endpoint disable(): %04x\n", trace->value);
      break;

    case TRACE_EPALLOCREQ:
      printf("Endpoint allocreq(): %04x\n", trace->value);
      break;

    case TRACE_EPFREEREQ:
      printf("Endpoint freereq(): %04x\n", trace->value);
      break;

    case TRACE_EPALLOCBUFFER:
      printf("Endpoint allocbuffer(): %04x\n", trace->value);
      break;

    case TRACE_EPFREEBUFFER:
      printf("Endpoint freebuffer(): %04x\n", trace->value);
      break;

    case TRACE_EPSUBMIT:
      printf("Endpoint submit(): %04x\n", trace->value);
      break;

    case TRACE_EPCANCEL:
      printf("Endpoint cancel(): %04x\n", trace->value);
      break;

    case TRACE_EPSTALL:
      printf("Endpoint stall(true): %04x\n", trace->value);
      break;

    case TRACE_EPRESUME:
      printf("Endpoint stall(false): %04x\n", trace->value);
      break;

    case TRACE_DEVALLOCEP:
      printf("Device allocep(): %04x\n", trace->value);
      break;

    case TRACE_DEVFREEEP:
      printf("Device freeep(): %04x\n", trace->value);
      break;

    case TRACE_DEVGETFRAME:
      printf("Device getframe(): %04x\n", trace->value);
      break;

    case TRACE_DEVWAKEUP:
      printf("Device wakeup(): %04x\n", trace->value);
      break;

    case TRACE_DEVSELFPOWERED:
      printf("Device selfpowered(): %04x\n", trace->value);
      break;

    case TRACE_DEVPULLUP:
      printf("Device pullup(): %04x\n", trace->value);
      break;

    case TRACE_CLASSBIND:
      printf("Class bind(): %04x\n", trace->value);
      break;

    case TRACE_CLASSUNBIND:
      printf("Class unbind(): %04x\n", trace->value);
      break;

    case TRACE_CLASSDISCONNECT:
      printf("Class disconnect(): %04x\n", trace->value);
      break;

    case TRACE_CLASSSETUP:
      printf("Class setup(): %04x\n", trace->value);
      break;

    case TRACE_CLASSSUSPEND:
      printf("Class suspend(): %04x\n", trace->value);
      break;

    case TRACE_CLASSRESUME:
      printf("Class resume(): %04x\n", trace->value);
      break;

    case TRACE_CLASSRDCOMPLETE:
      printf("Class RD request complete: %04x\n", trace->value);
      break;

    case TRACE_CLASSWRCOMPLETE:
      printf("Class WR request complete: %04x\n", trace->value);
      break;

    default:
      switch (TRACE_ID(trace->event))
        {
        case TRACE_CLASSAPI_ID:        /* Other class driver system API calls */
          printf("Class API call %d: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_CLASSSTATE_ID:      /* Track class driver state changes */
          printf("Class state %d: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTENTRY_ID:        /* Interrupt handler entry */
          printf("Interrupt %d entry: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTDECODE_ID:       /* Decoded interrupt trace->event */
          printf("Interrupt decode %d: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTEXIT_ID:         /* Interrupt handler exit */
          printf("Interrupt %d exit: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_OUTREQQUEUED_ID:    /* Request queued for OUT endpoint */
          printf("EP%d OUT request queued: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INREQQUEUED_ID:     /* Request queued for IN endpoint */
          printf("EP%d IN request queued: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_READ_ID:            /* Read (OUT) action */
          printf("EP%d OUT read: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_WRITE_ID:           /* Write (IN) action */
          printf("EP%d IN write: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_COMPLETE_ID:        /* Request completed */
          printf("EP%d request complete: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_DEVERROR_ID:        /* USB controller driver error event */
          printf("Controller error: %02x:%04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_CLSERROR_ID:        /* USB class driver error event */
          printf("Class error: %02x:%04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        default:
          printf("Unrecognized event: %02x:%02x:%04x\n",
                  TRACE_ID(trace->event) >> 8,
                  TRACE_DATA(trace->event), trace->value);
          break;
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: dumptrace
 *
 * Description:
 *   Dump collected trace data.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int dumptrace(void)
{
  int ret;

  ret =  usbtrace_enumerate(composite_enumerate, NULL);
  if (ret < 0)
    {
      printf("dumptrace: usbtrace_enumerate failed: %d\n", -ret);
    }

  return ret;
}
#else
#  define dumptrace() (OK)
#endif

/****************************************************************************
 * Name: open_serial
 ****************************************************************************/

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_DISABLE_SIGNALS)
static int open_serial(void)
{
  int errcode;
#ifdef CONFIG_USBDEV_TRACE
  int ret;
#endif

  /* Open the USB serial device for writing (blocking) */

  do
    {
      printf("open_serial: Opening USB serial driver\n");
      g_composite.outfd = open(CONFIG_SYSTEM_COMPOSITE_SERDEV, O_WRONLY);
      if (g_composite.outfd < 0)
        {
          errcode = errno;
          printf("open_serial: ERROR: Failed to open %s for writing: %d\n",
              CONFIG_SYSTEM_COMPOSITE_SERDEV, errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              printf("open_serial:        Not connected. Wait and try again.\n");
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              printf("open_serial:        Aborting\n");
              return -errcode;
            }
        }

      /* If USB tracing is enabled, then dump all collected trace data to
       * stdout.
       */

#ifdef CONFIG_USBDEV_TRACE
      ret = dumptrace();
      if (ret < 0)
        {
          return ret;
        }
#endif
    }
  while (g_composite.outfd < 0);

  /* Open the USB serial device for reading (non-blocking) */

  g_composite.infd = open(CONFIG_SYSTEM_COMPOSITE_SERDEV, O_RDONLY|O_NONBLOCK);
  if (g_composite.infd < 0)
    {
      errcode = errno;
      printf("open_serial: ERROR: Failed to open%s for reading: %d\n",
              CONFIG_SYSTEM_COMPOSITE_SERDEV, errcode);
      close(g_composite.outfd);
      return -errcode;
    }

  printf("open_serial: Successfully opened the serial driver\n");
  return OK;
}

/****************************************************************************
 * Name: echo_serial
 ****************************************************************************/

static int echo_serial(void)
{
  ssize_t bytesread;
  ssize_t byteswritten;
  int errcode;

  /* Read data */

  bytesread = read(g_composite.infd, g_composite.serbuf, CONFIG_SYSTEM_COMPOSITE_BUFSIZE);
  if (bytesread < 0)
    {
      errcode = errno;
      if (errcode != EAGAIN)
        {
          printf("echo_serial: ERROR: read failed: %d\n", errcode);
          return -errcode;
        }
      return OK;
    }

  /* Echo data */

  byteswritten = write(g_composite.outfd, g_composite.serbuf, bytesread);
  if (byteswritten < 0)
    {
      errcode = errno;
      printf("echo_serial: ERROR: write failed: %d\n", errcode);
      return -errcode;
    }
  else if (byteswritten != bytesread)
    {
      printf("echo_serial: ERROR: read size: %d write size: %d\n",
              bytesread, byteswritten);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: usbmsc_disconnect
 *
 * Description:
 *   Disconnect the USB MSC device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbmsc_disconnect(void)
{
  struct boardioc_usbdev_ctrl_s ctrl;

  ctrl.usbdev   = BOARDIOC_USBDEV_MSC;
  ctrl.action   = BOARDIOC_USBDEV_DISCONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &g_composite.mschandle;

  (void)boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mscclassobject
 *
 * Description:
 *   If the mass storage class driver is part of composite device, then
 *   its instantiation and configuration is a multi-step, board-specific,
 *   process (See comments for usbmsc_configure below).  In this case,
 *   board-specific logic must provide board_mscclassobject().
 *
 *   board_mscclassobject() is called from the composite driver.  It must
 *   encapsulate the instantiation and configuration of the mass storage
 *   class and the return the mass storage device's class driver instance
 *   to the composite driver.
 *
 * Input Parameters:
 *   classdev - The location to return the mass storage class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

int board_mscclassobject(FAR struct usbdevclass_driver_s **classdev)
{
  int ret;

  DEBUGASSERT(g_composite.mschandle == NULL);

  /* Initialize USB trace output IDs */

  usbtrace_enable(TRACE_BITSET);
  check_test_memory_usage("After usbtrace_enable()");

  /* Configure the mass storage device */

  printf("board_mscclassobject: Configuring with NLUNS=%d\n", CONFIG_SYSTEM_COMPOSITE_NLUNS);
  ret = usbmsc_configure(CONFIG_SYSTEM_COMPOSITE_NLUNS, &g_composite.mschandle);
  if (ret < 0)
    {
      printf("board_mscclassobject: usbmsc_configure failed: %d\n", -ret);
      return ret;
    }

  printf("board_mscclassobject: MSC handle=%p\n", g_composite.mschandle);
  check_test_memory_usage("After usbmsc_configure()");

  /* Bind the LUN(s) */

  printf("board_mscclassobject: Bind LUN=0 to %s\n", CONFIG_SYSTEM_COMPOSITE_DEVPATH1);
  ret = usbmsc_bindlun(g_composite.mschandle, CONFIG_SYSTEM_COMPOSITE_DEVPATH1, 0, 0, 0, false);
  if (ret < 0)
    {
      printf("board_mscclassobject: usbmsc_bindlun failed for LUN 1 using %s: %d\n",
               CONFIG_SYSTEM_COMPOSITE_DEVPATH1, -ret);
      usbmsc_disconnect();
      return ret;
    }

  check_test_memory_usage("After usbmsc_bindlun()");

#if CONFIG_SYSTEM_COMPOSITE_NLUNS > 1

  printf("board_mscclassobject: Bind LUN=1 to %s\n", CONFIG_SYSTEM_COMPOSITE_DEVPATH2);
  ret = usbmsc_bindlun(g_composite.mschandle, CONFIG_SYSTEM_COMPOSITE_DEVPATH2, 1, 0, 0, false);
  if (ret < 0)
    {
      printf("board_mscclassobject: usbmsc_bindlun failed for LUN 2 using %s: %d\n",
               CONFIG_SYSTEM_COMPOSITE_DEVPATH2, -ret);
      usbmsc_disconnect();
      return ret;
    }

  check_test_memory_usage("After usbmsc_bindlun() #2");

#if CONFIG_SYSTEM_COMPOSITE_NLUNS > 2

  printf("board_mscclassobject: Bind LUN=2 to %s\n", CONFIG_SYSTEM_COMPOSITE_DEVPATH3);
  ret = usbmsc_bindlun(g_composite.mschandle, CONFIG_SYSTEM_COMPOSITE_DEVPATH3, 2, 0, 0, false);
  if (ret < 0)
    {
      printf("board_mscclassobject: usbmsc_bindlun failed for LUN 3 using %s: %d\n",
               CONFIG_SYSTEM_COMPOSITE_DEVPATH3, -ret);
      usbmsc_disconnect();
      return ret;
    }

  check_test_memory_usage("After usbmsc_bindlun() #3");

#endif
#endif

  /* Get the mass storage device's class object */

  ret = usbmsc_classobject(g_composite.mschandle, classdev);
  if (ret < 0)
    {
      printf("board_mscclassobject: usbmsc_classobject failed: %d\n", -ret);
      usbmsc_disconnect();
    }

  check_test_memory_usage("After usbmsc_classobject()");
  return ret;
}

/****************************************************************************
 * Name: board_mscuninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This is just an application-
 *   specific wrapper aboutn usbmsc_unitialize() that is called form the composite
 *   device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_mscclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_mscuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  DEBUGASSERT(g_composite.mschandle != NULL);
  usbmsc_disconnect();
}

/****************************************************************************
 * Name: board_cdcclassobject
 *
 * Description:
 *   If the CDC serial class driver is part of composite device, then
 *   board-specific logic must provide board_cdcclassobject().  In the simplest
 *   case, board_cdcclassobject() is simply a wrapper around cdcacm_classobject()
 *   that provides the correct device minor number.
 *
 * Input Parameters:
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

int board_cdcclassobject(FAR struct usbdevclass_driver_s **classdev)
{
  int ret;

  /* Initialize the USB serial driver */

  printf("board_cdcclassobject: Initializing USB serial driver\n");
  ret = cdcacm_classobject(CONFIG_SYSTEM_COMPOSITE_TTYUSB, classdev);
  if (ret < 0)
    {
      printf("board_cdcclassobject: ERROR: Failed to create the USB serial device: %d\n", -ret);
    }

  return ret;
}

/****************************************************************************
 * Name: board_cdcuninitialize
 *
 * Description:
 *   Un-initialize the USB serial class driver.  This is just an application-
 *   specific wrapper aboutn cdcadm_unitialize() that is called form the composite
 *   device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_cdcclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_cdcuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  struct boardioc_usbdev_ctrl_s ctrl;

  DEBUGASSERT(classdev != NULL);

  ctrl.usbdev   = BOARDIOC_USBDEV_CDCACM;
  ctrl.action   = BOARDIOC_USBDEV_DISCONNECT;
  ctrl.instance = CONFIG_SYSTEM_COMPOSITE_TTYUSB;
  ctrl.handle   = (FAR void **)&classdev;

  (void)boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
}

/****************************************************************************
 * conn_main
 *
 * Description:
 *   This is the main program that configures the USB mass storage device
 *   and exports the LUN(s).  If CONFIG_NSH_BUILTIN_APPS is defined
 *   in the NuttX configuration, then this program can be executed by
 *   entering the "msconn" command at the NSH console.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int conn_main(int argc, char *argv[])
#endif
{
  struct boardioc_usbdev_ctrl_s ctrl;
  int ret;

  /* If this program is implemented as the NSH 'msconn' command, then we need to
   * do a little error checking to assure that we are not being called re-entrantly.
   */

#ifdef CONFIG_NSH_BUILTIN_APPS
   /* Check if there is a non-NULL USB mass storage device handle (meaning that the
    * USB mass storage device is already configured).
    */

   if (g_composite.cmphandle)
     {
       printf("conn_main: ERROR: Already connected\n");
       return 1;
     }
#endif

#ifdef CONFIG_SYSTEM_COMPOSITE_DEBUGMM
#  ifdef CONFIG_CAN_PASS_STRUCTS
  g_composite.mmstart    = mallinfo();
  g_composite.mmprevious = g_composite.mmstart;
#  else
  (void)mallinfo(&g_composite.mmstart);
  memcpy(&g_composite.mmprevious, &g_composite.mmstart, sizeof(struct mallinfo));
#  endif
#endif

  /* Perform architecture-specific initialization */

  printf("conn_main: Performing architecture-specific initialization\n");

  ctrl.usbdev   = BOARDIOC_USBDEV_COMPOSITE;
  ctrl.action   = BOARDIOC_USBDEV_INITIALIZE;
  ctrl.instance = 0;
  ctrl.handle   = NULL;

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  if (ret < 0)
    {
      printf("conn_main: boardctl(BOARDIOC_USBDEV_CONTROL) failed: %d\n", -ret);
      return 1;
    }

  check_test_memory_usage("After boardctl(BOARDIOC_USBDEV_CONTROL)");

  /* Initialize the USB composite device device */

  ctrl.usbdev   = BOARDIOC_USBDEV_COMPOSITE;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &g_composite.cmphandle;

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  if (ret < 0)
    {
      printf("conn_main: boardctl(BOARDIOC_USBDEV_CONTROL) failed: %d\n", -ret);
      return 1;
    }

  check_test_memory_usage("After boardctl(BOARDIOC_USBDEV_CONTROL)");

#if defined(CONFIG_USBDEV_TRACE) && CONFIG_USBDEV_TRACE_INITIALIDSET != 0
  /* If USB tracing is enabled and tracing of initial USB events is specified,
   * then dump all collected trace data to stdout
   */

  sleep(5);
  ret = dumptrace();
  if (ret < 0)
    {
      goto errout_bad_dump;
    }
#endif

  /* It this program was configued as an NSH command, then just exit now.
   * Also, if signals are not enabled (and, hence, sleep() is not supported.
   * then we have not real option but to exit now.
   */

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_DISABLE_SIGNALS)

  /* Otherwise, this thread will hang around and monitor the USB activity */

  /* Open the serial driver */

  ret = open_serial();
  if (ret < 0)
    {
      goto errout;
    }

  /* Now looping */

  for (;;)
    {
      /* Sleep for a bit */

      fflush(stdout);
      sleep(5);

      /* Echo any serial data */

      ret = echo_serial();
      if (ret < 0)
        {
          goto errout;
        }

      /* Dump trace data */

#  ifdef CONFIG_USBDEV_TRACE
      printf("\n" "conn_main: USB TRACE DATA:\n");
      ret = dumptrace();
      if (ret < 0)
        {
          goto errout;
        }

      check_test_memory_usage("After usbtrace_enumerate()");
#  else
      printf("conn_main: Still alive\n");
#  endif
    }
#else

   printf("conn_main: Connected\n");
   check_test_memory_usage("After composite device connection");
#endif

   /* Dump debug memory usage */

   printf("conn_main: Exiting\n");
#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_DISABLE_SIGNALS)
   close(g_composite.infd);
   close(g_composite.outfd);
#endif
#ifdef CONFIG_NSH_BUILTIN_APPS
#endif
   final_memory_usage("Final memory usage");
   return 0;

#if defined(CONFIG_USBDEV_TRACE) && CONFIG_USBDEV_TRACE_INITIALIDSET != 0
errout_bad_dump:
#endif

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_DISABLE_SIGNALS)
errout:
  close(g_composite.infd);
  close(g_composite.outfd);
#endif

  ctrl.usbdev   = BOARDIOC_USBDEV_COMPOSITE;
  ctrl.action   = BOARDIOC_USBDEV_DISCONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &g_composite.cmphandle;

  (void)boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  final_memory_usage("Final memory usage");
  return 1;
}

/****************************************************************************
 * disconn_main
 *
 * Description:
 *   This is a program entry point that will disconnect the USB mass storage
 *   device.  This program is only available if CONFIG_NSH_BUILTIN_APPS
 *   is defined in the NuttX configuration.  In that case, this program can
 *   be executed by entering the "msdis" command at the NSH console.
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char **argv)
#else
int disconn_main(int argc, char *argv[])
#endif
{
  struct boardioc_usbdev_ctrl_s ctrl;

  /* First check if the USB mass storage device is already connected */

  if (!g_composite.cmphandle)
    {
      printf("disconn_main: ERROR: Not connected\n");
      return 1;
    }

  check_test_memory_usage("Since MS connection");

  /* Then disconnect the device and uninitialize the USB mass storage driver */

  ctrl.usbdev   = BOARDIOC_USBDEV_COMPOSITE;
  ctrl.action   = BOARDIOC_USBDEV_DISCONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &g_composite.cmphandle;

  (void)boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);

  g_composite.cmphandle = NULL;
  printf("disconn_main: Disconnected\n");
  check_test_memory_usage("After boardctl(BOARDIOC_USBDEV_CONTROL)");

  /* Dump debug memory usage */

  final_memory_usage("Final memory usage");
  return 0;
}
#endif
