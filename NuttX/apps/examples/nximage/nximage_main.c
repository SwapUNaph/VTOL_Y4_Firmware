/****************************************************************************
 * examples/nximage/nximage_main.c
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#ifdef CONFIG_NX_LCDDRIVER
#  include <nuttx/lcd/lcd.h>
#else
#  include <nuttx/video/fb.h>
#  ifdef CONFIG_VNCSERVER
#    include <nuttx/video/vnc.h>
#  endif
#endif

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nximage.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* If not specified, assume that the hardware supports one video plane */

#ifndef CONFIG_EXAMPLES_NXIMAGE_VPLANE
#  define CONFIG_EXAMPLES_NXIMAGE_VPLANE 0
#endif

/* If not specified, assume that the hardware supports one LCD device */

#ifndef CONFIG_EXAMPLES_NXIMAGE_DEVNO
#  define CONFIG_EXAMPLES_NXIMAGE_DEVNO 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  NXEXIT_SUCCESS /* exit code */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_initialize
 *
 * Description:
 *   Initialize the LCD or framebuffer device (single user mode only), then
 *   open NX.
 *
 ****************************************************************************/

static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  int ret;

#if defined(CONFIG_EXAMPLES_NXIMAGE_EXTERNINIT)
  struct boardioc_graphics_s devinfo;

  /* Use external graphics driver initialization */

  printf("nximage_initialize: Initializing external graphics device\n");

  devinfo.devno = CONFIG_EXAMPLES_NXIMAGE_DEVNO;
  devinfo.dev = NULL;

  ret = boardctl(BOARDIOC_GRAPHICS_SETUP, (uintptr_t)&devinfo);
  if (ret < 0)
    {
      printf("nximage_initialize: boardctl failed, devno=%d: %d\n",
             CONFIG_EXAMPLES_NXIMAGE_DEVNO, errno);
      g_nximage.code = NXEXIT_EXTINITIALIZE;
      return ERROR;
    }

  dev = devinfo.dev;

#elif defined(CONFIG_NX_LCDDRIVER)
  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      g_nximage.code = NXEXIT_LCDINITIALIZE;
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_NXIMAGE_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_NXIMAGE_DEVNO);
      g_nximage.code = NXEXIT_LCDGETDEV;
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));
#else
  /* Initialize the frame buffer device */

  printf("nximage_initialize: Initializing framebuffer\n");

  ret = up_fbinitialize(0);
  if (ret < 0)
    {
      printf("nximage_initialize: up_fbinitialize failed: %d\n", -ret);

      g_nximage.code = NXEXIT_FBINITIALIZE;
      return ERROR;
    }

  dev = up_fbgetvplane(0, CONFIG_EXAMPLES_NXIMAGE_VPLANE);
  if (!dev)
    {
      printf("nximage_initialize: up_fbgetvplane failed, vplane=%d\n",
             CONFIG_EXAMPLES_NXIMAGE_VPLANE);

      g_nximage.code = NXEXIT_FBGETVPLANE;
      return ERROR;
    }
#endif

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_nximage.hnx = nx_open(dev);
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);

      g_nximage.code = NXEXIT_NXOPEN;
      return ERROR;
    }

#ifdef CONFIG_VNCSERVER
  /* Setup the VNC server to support keyboard/mouse inputs */

  ret = vnc_default_fbinitialize(0, g_nximage.hnx);
  if (ret < 0)
    {
      printf("vnc_default_fbinitialize failed: %d\n", ret);

      g_nximage.code = NXEXIT_FBINITIALIZE;
      nx_close(g_nximage.hnx);
      return ERROR;
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_main
 *
 * Description:
 *   Main entry pointer.  Configures the basic display resources.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int nximage_main(int argc, char *argv[])
#endif
{
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize NX */

  ret = nximage_initialize();
  printf("nximage_main: NX handle=%p\n", g_nximage.hnx);
  if (!g_nximage.hnx || ret < 0)
    {
      printf("nximage_main: Failed to get NX handle: %d\n", errno);
      g_nximage.code = NXEXIT_NXOPEN;
      goto errout;
    }

  /* Set the background to the configured background color */

  color =  nximage_bgcolor();
  printf("nximage_main: Set background color=%u\n", color);

  ret = nx_setbgcolor(g_nximage.hnx, &color);
  if (ret < 0)
    {
      printf("nximage_main: nx_setbgcolor failed: %d\n", errno);
      g_nximage.code = NXEXIT_NXSETBGCOLOR;
      goto errout_with_nx;
    }

  /* Get the background window */

  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_main: nx_setbgcolor failed: %d\n", errno);
      g_nximage.code = NXEXIT_NXREQUESTBKGD;
      goto errout_with_nx;
    }

  /* Wait until we have the screen resolution.  We'll have this immediately
   * unless we are dealing with the NX server.
   */

  while (!g_nximage.havepos)
    {
      (void)sem_wait(&g_nximage.sem);
    }
  printf("nximage_main: Screen resolution (%d,%d)\n", g_nximage.xres, g_nximage.yres);

  /* Now, put up the NuttX logo and wait a bit so that it visible. */

  nximage_image(g_nximage.hbkgd);
  sleep(5);

  /* Release background */

  (void)nx_releasebkgd(g_nximage.hbkgd);

  /* Close NX */

errout_with_nx:
  printf("nximage_main: Close NX\n");
  nx_close(g_nximage.hnx);
errout:
  return g_nximage.code;
}
