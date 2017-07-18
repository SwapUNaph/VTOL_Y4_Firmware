/****************************************************************************
 * configs/ea3152/src/lpc31_usbmsc.c
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Configure and register the SAM3U MMC/SD SDIO block driver.
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <stdlib.h>

#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/mkfatfs.h>
#include <nuttx/drivers/ramdisk.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

#ifndef CONFIG_SYSTEM_USBMSC_DEVPATH1
#  define CONFIG_SYSTEM_USBMSC_DEVPATH1  "/dev/ram"
#endif

static const char g_source[] = CONFIG_SYSTEM_USBMSC_DEVPATH1;
static struct fat_format_s g_fmt = FAT_FORMAT_INITIALIZER;

#define USBMSC_NSECTORS        64
#define USBMSC_SECTORSIZE      512
#define BUFFER_SIZE            (USBMSC_NSECTORS*USBMSC_SECTORSIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Perform architecture specific initialization of the USB MSC device.
 *
 ****************************************************************************/

int board_usbmsc_initialize(int port)
{
  uint8_t *pbuffer;
  int ret;

  pbuffer = (uint8_t *)kmm_malloc(BUFFER_SIZE);
  if (!pbuffer)
    {
      err("ERROR: board_usbmsc_initialize: Failed to allocate ramdisk of size %d\n",
          BUFFER_SIZE);
      return -ENOMEM;
    }

  /* Register a RAMDISK device to manage this RAM image */

  ret = ramdisk_register(CONFIG_SYSTEM_USBMSC_DEVMINOR1,
                         pbuffer,
                         USBMSC_NSECTORS,
                         USBMSC_SECTORSIZE,
                         RDFLAG_WRENABLED | RDFLAG_FUNLINK);
  if (ret < 0)
    {
      err("ERROR: create_ramdisk: Failed to register ramdisk at %s: %d\n",
          g_source, -ret);
      kmm_free(pbuffer);
      return ret;
    }

  /* Create a FAT filesystem on the ramdisk */

  ret = mkfatfs(g_source, &g_fmt);
  if (ret < 0)
    {
      err("ERROR: create_ramdisk: Failed to create FAT filesystem on ramdisk at %s\n",
          g_source);
      /* kmm_free(pbuffer); -- RAM disk is registered */
      return ret;
    }

  return 0;
}
