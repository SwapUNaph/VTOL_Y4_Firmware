/****************************************************************************
 * apps/nshlib/nsh_telnetd.c
 *
 *   Copyright (C) 2007-2013, 2016 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <stdio.h>
#include <debug.h>

#include <arpa/inet.h>

#include "netutils/telnetd.h"

#include "nsh.h"
#include "nsh_console.h"

#ifdef CONFIG_NSH_TELNET

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_telnetmain
 ****************************************************************************/

static int nsh_telnetmain(int argc, char *argv[])
{
  FAR struct console_stdio_s *pstate = nsh_newconsole();
  FAR struct nsh_vtbl_s *vtbl;

  DEBUGASSERT(pstate != NULL);
  vtbl = &pstate->cn_vtbl;

  _info("Session [%d] Started\n", getpid());

#ifdef CONFIG_NSH_TELNET_LOGIN
  /* Login User and Password Check */

  if (nsh_telnetlogin(pstate) != OK)
    {
      nsh_exit(vtbl, 1);
      return -1; /* nsh_exit does not return */
    }
#endif /* CONFIG_NSH_TELNET_LOGIN */

  /* The following logic mostly the same as the login in nsh_session.c.  It
   * differs only in that gets() is called to get the command instead of
   * readline().
   */

  /* Present a greeting and possibly a Message of the Day (MOTD) */

  fputs(g_nshgreeting, pstate->cn_outstream);

#ifdef CONFIG_NSH_MOTD
# ifdef CONFIG_NSH_PLATFORM_MOTD
  /* Output the platform message of the day */

  platform_motd(vtbl->iobuffer, IOBUFFERSIZE);
  fprintf(pstate->cn_outstream, "%s/n", vtbl->iobuffer);

# else
  /* Output the fixed message of the day */

  fprintf(pstate->cn_outstream, "%s/n", g_nshmotd);
# endif
#endif

  fflush(pstate->cn_outstream);

  /* Execute the startup script.  If standard console is also defined, then
   * we will not bother with the initscript here (although it is safe to
   * call nshinitscript multiple times).
   */

#if defined(CONFIG_NSH_ROMFSETC) && !defined(CONFIG_NSH_CONSOLE)
  (void)nsh_initscript(vtbl);
#endif

  /* Execute the login script */

#ifdef CONFIG_NSH_ROMFSRC
  (void)nsh_loginscript(vtbl);
#endif

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* Display the prompt string */

      fputs(g_nshprompt, pstate->cn_outstream);
      fflush(pstate->cn_outstream);

      /* Get the next line of input from the Telnet client */

      if (fgets(pstate->cn_line, CONFIG_NSH_LINELEN, INSTREAM(pstate)) != NULL)
        {
          /* Parse process the received Telnet command */

          (void)nsh_parse(vtbl, pstate->cn_line);
          fflush(pstate->cn_outstream);
        }
      else
        {
          fprintf(pstate->cn_outstream, g_fmtcmdfailed, "nsh_telnetmain",
                  "fgets", NSH_ERRNO);
          nsh_exit(vtbl, 1);
        }
    }

  /* Clean up */

  nsh_exit(vtbl, 0);

  /* We do not get here, but this is necessary to keep some compilers happy */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_telnetstart
 *
 * Description:
 *   nsh_telnetstart() starts the Telnet daemon that will allow multiple
 *   NSH connections via Telnet.  This function returns immediately after
 *   the daemon has been started.
 *
 * Input Parameters:
 *   None.  All of the properties of the Telnet daemon are controlled by
 *   NuttX configuration setting.
 *
 * Returned Values:
 *   The task ID of the Telnet daemon was successfully started.  A negated
 *   errno value will be returned on failure.
 *
 ****************************************************************************/

int nsh_telnetstart(void)
{
  struct telnetd_config_s config;
  int ret;

  /* Initialize any USB tracing options that were requested.  If standard
   * console is also defined, then we will defer this step to the standard
   * console.
   */

#if defined(CONFIG_NSH_USBDEV_TRACE) && !defined(CONFIG_NSH_CONSOLE)
  usbtrace_enable(TRACE_BITSET);
#endif

  /* Configure the telnet daemon */

  config.d_port      = HTONS(CONFIG_NSH_TELNETD_PORT);
  config.d_priority  = CONFIG_NSH_TELNETD_DAEMONPRIO;
  config.d_stacksize = CONFIG_NSH_TELNETD_DAEMONSTACKSIZE;
  config.t_priority  = CONFIG_NSH_TELNETD_CLIENTPRIO;
  config.t_stacksize = CONFIG_NSH_TELNETD_CLIENTSTACKSIZE;
  config.t_entry     = nsh_telnetmain;

  /* Start the telnet daemon */

  _info("Starting the Telnet daemon\n");
  ret = telnetd_start(&config);
  if (ret < 0)
    {
      _err("ERROR: Failed to tart the Telnet daemon: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_NSH_TELNET */
