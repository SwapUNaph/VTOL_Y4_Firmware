/****************************************************************************
 * examples/usbterm/usbterm_main.c
 *
 *   Copyright (C) 2011-2013, 2015-2016 Gregory Nutt. All rights reserved.
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
#include <sys/stat.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "system/readline.h"

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/pl2303.h>
#endif

#include "usbterm.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* USB terminal state data */

struct usbterm_globals_s g_usbterm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trace_callback
 *
 * Description:
 *   Callback from USB trace instrumentation.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int trace_callback(struct usbtrace_s *trace, void *arg)
{
  usbtrace_trprintf((trprintf_t)trmessage, trace->event, trace->value);
  return 0;
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
static void dumptrace(void)
{
  (void)usbtrace_enumerate(trace_callback, NULL);
}
#else
#  define dumptrace()
#endif

/****************************************************************************
 * Name: usbterm_listener
 *
 * Description:
 *   Entry point for the listener thread.
 *
 ****************************************************************************/

static FAR void *usbterm_listener(FAR void *parameter)
{
  printf("usbterm_listener: Waiting for remote input\n");
  for (;;)
    {
      /* Display the prompt string on the remote USB serial connection -- only
       * if we know that there is someone listening at the other end.  The
       * remote side must initiate the the conversation.
       */

      if (g_usbterm.peer)
        {
          fputs("\rusbterm> ", g_usbterm.outstream);
          fflush(g_usbterm.outstream);
        }

      /* Get the next line of input from the remote USB serial connection */

      if (fgets(g_usbterm.inbuffer, CONFIG_EXAMPLES_USBTERM_BUFLEN, g_usbterm.instream))
        {
          /* If we receive anything, then we can be assured that there is someone
           * with the serial driver open on the remote host.
           */

          g_usbterm.peer = true;

          /* Echo the line on the local stdout */

          fputs(g_usbterm.inbuffer, stdout);

          /* Display the prompt string on stdout */

          fputs("usbterm> ", stdout);
          fflush(stdout);
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }

  /* Won't get here */

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbterm_main
 *
 * Description:
 *   Main entry point for the USB serial terminal example.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int usbterm_main(int argc, char *argv[])
#endif
{
  struct boardioc_usbdev_ctrl_s ctrl;
  FAR void *handle;
  pthread_attr_t attr;
  int ret;

  /* Initialize global data */

  memset(&g_usbterm, 0, sizeof(struct usbterm_globals_s));

  /* Initialization of the USB hardware may be performed by logic external to
   * this test.
   */

#ifdef CONFIG_EXAMPLES_USBTERM_DEVINIT
  printf("usbterm_main: Performing external device initialization\n");
  ret = usbterm_devinit();
  if (ret != OK)
    {
      printf("usbterm_main: usbterm_devinit failed: %d\n", ret);
      goto errout;
    }
#endif

  /* Initialize the USB serial driver */

  printf("usbterm_main: Registering USB serial driver\n");

#ifdef CONFIG_CDCACM

  ctrl.usbdev   = BOARDIOC_USBDEV_CDCACM;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &handle;

#else

  ctrl.usbdev   = BOARDIOC_USBDEV_PL2303;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle   = &handle;

#endif

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  if (ret < 0)
    {
      printf("usbterm_main: ERROR: Failed to create the USB serial device: %d\n", -ret);
      goto errout_with_devinit;
    }

  printf("usbterm_main: Successfully registered the serial driver\n");

#if defined(CONFIG_USBDEV_TRACE) && CONFIG_USBDEV_TRACE_INITIALIDSET != 0
  /* If USB tracing is enabled and tracing of initial USB events is specified,
   * then dump all collected trace data to stdout
   */

  sleep(5);
  dumptrace();
#endif

  /* Then, in any event, configure trace data collection as configured */

  usbtrace_enable(TRACE_BITSET);

  /* Open the USB serial device for writing */

  do
    {
      printf("usbterm_main: Opening USB serial driver\n");

      g_usbterm.outstream = fopen(USBTERM_DEVNAME, "w");
      if (g_usbterm.outstream == NULL)
        {
          int errcode = errno;
          printf("usbterm_main: ERROR: Failed to open " USBTERM_DEVNAME " for writing: %d\n",
                 errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              printf("usbterm_main:        Not connected. Wait and try again.\n");
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              goto errout_with_devinit;
            }
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }
  while (g_usbterm.outstream == NULL);

  /* Open the USB serial device for reading.  Since we are already connected, this
   * should not fail.
   */

  g_usbterm.instream = fopen(USBTERM_DEVNAME, "r");
  if (g_usbterm.instream == NULL)
    {
      printf("usbterm_main: ERROR: Failed to open " USBTERM_DEVNAME " for reading: %d\n", errno);
      goto errout_with_outstream;
    }

  printf("usbterm_main: Successfully opened the serial driver\n");

  /* Start the USB term listener thread */

  printf("usbterm_main: Starting the listener thread\n");

  ret = pthread_attr_init(&attr);
  if (ret != OK)
    {
      printf("usbterm_main: pthread_attr_init failed: %d\n", ret);
      goto errout_with_streams;
    }

  ret = pthread_create(&g_usbterm.listener, &attr,
                       usbterm_listener, (pthread_addr_t)0);
  if (ret != 0)
    {
      printf("usbterm_main: Error in thread creation: %d\n", ret);
      goto errout_with_streams;
    }

  /* Send messages and get responses -- forever */

  printf("usbterm_main: Waiting for local input\n");
  for (;;)
    {
      /* Display the prompt string on stdout */

      fputs("usbterm> ", stdout);
      fflush(stdout);

      /* Get the next line of input */

#ifdef CONFIG_EXAMPLES_USBTERM_FGETS
      /* fgets returns NULL on end-of-file or any I/O error */

      if (fgets(g_usbterm.outbuffer, CONFIG_EXAMPLES_USBTERM_BUFLEN, stdin) == NULL)
        {
          printf("ERROR: fgets failed: %d\n", errno);
          return 1;
        }
#else
      ret = readline(g_usbterm.outbuffer, CONFIG_EXAMPLES_USBTERM_BUFLEN, stdin, stdout);

      /* Readline normally returns the number of characters read,
       * but will return EOF on end of file or if an error occurs.  Either
       * will cause the session to terminate.
       */

      if (ret == EOF)
        {
          printf("ERROR: readline failed: %d\n", ret);
          return 1;
        }
#endif
      /* Is there anyone listening on the other end? */

      else if (g_usbterm.peer)
        {
          /* Yes.. Send the line of input via USB */

          fputs(g_usbterm.outbuffer, g_usbterm.outstream);

          /* Display the prompt string on the remote USB serial connection */

          fputs("\rusbterm> ", g_usbterm.outstream);
          fflush(g_usbterm.outstream);
        }
      else
        {
          printf("Still waiting for remote peer.  Please try again later.\n");
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }

  /* Error exits */

errout_with_streams:
  fclose(g_usbterm.instream);
errout_with_outstream:
  fclose(g_usbterm.outstream);
errout_with_devinit:
#ifdef CONFIG_EXAMPLES_USBTERM_DEVINIT
  usbterm_devuninit();
errout:
#endif
  printf("usbterm_main:        Aborting\n");
  return 1;
}

