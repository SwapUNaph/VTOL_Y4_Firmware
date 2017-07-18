/****************************************************************************
 * apps/nshlib/nsh_syscmds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <sys/boardctl.h>
#include <sys/utsname.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_CUSTOM
#  ifndef CONFIG_ARCH_BOARD_CUSTOM_NAME
#    define BOARD_NAME g_unknown
#  else
#    define BOARD_NAME CONFIG_ARCH_BOARD_CUSTOM_NAME
#  endif
#else
#  ifndef CONFIG_ARCH_BOARD
#    define BOARD_NAME g_unknown
#  else
#    define BOARD_NAME CONFIG_ARCH_BOARD
#  endif
#endif

#define UNAME_KERNEL   (1 << 0)
#define UNAME_NODE     (1 << 1)
#define UNAME_RELEASE  (1 << 2)
#define UNAME_VERISON  (1 << 3)
#define UNAME_MACHINE  (1 << 4)
#define UNAME_PLATFORM (1 << 5)
#define UNAME_UNKNOWN  (1 << 6)

#ifdef CONFIG_NET
#  define UNAME_ALL    (UNAME_KERNEL | UNAME_NODE | UNAME_RELEASE | \
                        UNAME_VERISON | UNAME_MACHINE | UNAME_PLATFORM)
#else
#  define UNAME_ALL    (UNAME_KERNEL | UNAME_RELEASE | UNAME_VERISON | \
                        UNAME_MACHINE | UNAME_PLATFORM)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_unknown[] = "unknown";

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_shutdown
 ****************************************************************************/

#if (defined(CONFIG_BOARDCTL_POWEROFF) || defined(CONFIG_BOARDCTL_RESET)) && \
    !defined(CONFIG_NSH_DISABLE_SHUTDOWN)

int cmd_shutdown(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
#if defined(CONFIG_BOARDCTL_POWEROFF) && defined(CONFIG_BOARDCTL_RESET)
  /* If both shutdown and reset are supported, then a single option may
   * be provided to select the reset behavior (--reboot).  We know here
   * that argc is either 1 or 2.
   */

  if (argc == 2)
    {
      /* Verify that the single argument is --reboot */

      if (strcmp(argv[1], "--reboot") != 0)
        {
          nsh_output(vtbl, g_fmtarginvalid, argv[0]);
          return ERROR;
        }

      /* Invoke the BOARDIOC_RESET board control to reset the board.  If
       * the board_reset() function returns, then it was not possible to
       * reset the board due to some constraints.
       */

      (void)boardctl(BOARDIOC_RESET, EXIT_SUCCESS);
    }
  else
    {
      /* Invoke the BOARDIOC_POWEROFF board control to shutdown the board.
       * If the board_power_off function returns, then it was not possible
       * to power-off the* board due to some constraints.
       */

      (void)boardctl(BOARDIOC_POWEROFF, EXIT_SUCCESS);
    }

#elif defined(CONFIG_BOARDCTL_RESET)
  /* Only reset behavior is supported and we already know that exactly one
   * argument has been provided.
   */

  /* Verify that the single argument is --reboot */

  if (strcmp(argv[1], "--reboot") != 0)
    {
      nsh_output(vtbl, g_fmtarginvalid, argv[0]);
      return ERROR;
    }

  /* Invoke the BOARDIOC_RESET board control to reset the board.  If
   * the board_reset() function returns, then it was not possible to
   * reset the board due to some constraints.
   */

  (void)boardctl(BOARDIOC_RESET, EXIT_SUCCESS);

#else
  /* Only the reset behavior is supported and we already know that there is
   * no argument to the command.
   */

  /* Invoke the BOARDIOC_POWEROFF board control to shutdown the board.  If
   * the board_power_off function returns, then it was not possible to power-
   * off the board due to some constraints.
   */

  (void)boardctl(BOARDIOC_POWEROFF, EXIT_SUCCESS);
#endif

  /* boarctl() will not return in any case.  It if does, it means that
   * there was a problem with the shutdown/resaet operaion.
   */

  nsh_output(vtbl, g_fmtcmdfailed, argv[0], "boardctl", NSH_ERRNO);
  return ERROR;
}
#endif /* CONFIG_BOARDCTL_POWEROFF && !CONFIG_NSH_DISABLE_SHUTDOWN */

/****************************************************************************
 * Name: cmd_poweroff
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL_POWEROFF) && !defined(CONFIG_NSH_DISABLE_POWEROFF)
int cmd_poweroff(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  /* Invoke the BOARDIOC_POWEROFF board control to shutdown the board.  If
   * the board_power_off function returns, then it was not possible to power-
   * off the board due to some constraints.
   */

  (void)boardctl(BOARDIOC_POWEROFF, EXIT_SUCCESS);

  /* boarctl() will not return in any case.  It if does, it means that
   * there was a problem with the shutdown operaion.
   */

  nsh_output(vtbl, g_fmtcmdfailed, argv[0], "boardctl", NSH_ERRNO);
  return ERROR;
}
#endif

/****************************************************************************
 * Name: cmd_reboot
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL_RESET) && !defined(CONFIG_NSH_DISABLE_REBOOT)
int cmd_reboot(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  /* Invoke the BOARDIOC_RESET board control to reset the board.  If
   * the board_reset() function returns, then it was not possible to
   * reset the board due to some constraints.
   */

  (void)boardctl(BOARDIOC_RESET, EXIT_SUCCESS);

  /* boarctl() will not return in this case.  It if does, it means that
   * there was a problem with the reset operaion.
   */

  nsh_output(vtbl, g_fmtcmdfailed, argv[0], "boardctl", NSH_ERRNO);
  return ERROR;
}
#endif

/****************************************************************************
 * Name: cmd_uname
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_UNAME
int cmd_uname(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR const char *str;
  struct utsname info;
  unsigned int set;
  int option;
  bool badarg;
  bool first;
  int ret;
  int i;

  /* Get the ping options */

  set    = 0;
  badarg = false;

  while ((option = getopt(argc, argv, "asonrvmpi")) != ERROR)
    {
      switch (option)
        {
          case 'a':
            set = UNAME_ALL;
            break;

          case 'o':
          case 's':
            set |= UNAME_KERNEL;
            break;

#ifdef CONFIG_NET
          case 'n':
            set |= UNAME_NODE;
            break;
#endif

          case 'r':
            set |= UNAME_RELEASE;
            break;

          case 'v':
            set |= UNAME_VERISON;
            break;

          case 'm':
            set |= UNAME_MACHINE;
            break;

          case 'p':
            if (set != UNAME_ALL)
              {
                set |= UNAME_UNKNOWN;
              }
            break;

          case 'i':
            set |= UNAME_PLATFORM;
            break;

          case '?':
          default:
            nsh_output(vtbl, g_fmtarginvalid, argv[0]);
            badarg = true;
            break;
        }
    }

  /* If a bad argument was encountered, then return without processing the
   * command
   */

  if (badarg)
    {
      return ERROR;
    }

  /* If nothing is provided on the command line, the default is -s */

  if (set == 0)
    {
      set = UNAME_KERNEL;
    }

  /* Get uname data */

  ret = uname(&info);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "uname", NSH_ERRNO);
      return ERROR;
    }

  /* Process each option */

  first = true;
  for (i = 0; set != 0; i++)
    {
      unsigned int mask = (1 << i);
      if ((set & mask) != 0)
        {
          set &= ~mask;
          switch (i)
            {
              case 0: /* print the kernel/operating system name */
                str = info.sysname;
                break;

#ifdef CONFIG_NET
              case 1: /* Print noname */
                str = info.nodename;
                break;
#endif
              case 2: /* Print the kernel release */
                str = info.release;
                break;

              case 3: /* Print the kernel version */
                str = info.version;
                break;

              case 4: /* Print the machine hardware name */
                str = info.machine;
                break;

              case 5: /* Print the machine platform name */
                str = BOARD_NAME;
                break;

              case 6: /* Print "unknown" */
                str = g_unknown;
                break;

              default:
                nsh_output(vtbl, g_fmtarginvalid, argv[0]);
                return ERROR;
            }

          if (!first)
            {
              nsh_output(vtbl, " ");
            }

          nsh_output(vtbl, str);
          first = false;
        }
    }

  nsh_output(vtbl, "\n");
  return OK;
}
#endif

