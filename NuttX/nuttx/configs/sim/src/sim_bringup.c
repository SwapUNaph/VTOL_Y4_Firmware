/****************************************************************************
 * configs/sim/src/sam_bringup.c
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/timers/oneshot.h>

#include "up_internal.h"
#include "sim.h"

#ifdef CONFIG_GRAPHICS_TRAVELER_ROMFSDEMO
int trv_mount_world(int minor, FAR const char *mountpoint);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int sim_bringup(void)
{
#ifdef CONFIG_ONESHOT
  FAR struct oneshot_lowerhalf_s *oneshot;
#endif
#if defined(CONFIG_FS_PROCFS) || defined(CONFIG_ONESHOT)
  int ret;
#endif

#ifdef CONFIG_LIB_ZONEINFO_ROMFS
  /* Mount the TZ database */

  (void)sim_zoneinfo(3);
#endif

#ifdef CONFIG_EXAMPLES_GPIO
  /* Initialize simulated GPIO drivers */

  (void)sim_gpio_initialize();
#endif

#ifdef CONFIG_ONESHOT
  /* Get an instance of the simulated oneshot timer */

  oneshot = oneshot_initialize(0, 0);
  if (oneshot == NULL)
    {
      _err("ERROR: oneshot_initialize faile\n");
    }
  else
    {
#ifdef CONFIG_CPULOAD_ONESHOT
      /* Configure the oneshot timer to support CPU load measurement */

      sched_oneshot_extclk(oneshot);

#else
      /* Initialize the simulated oneshot driver */

      ret = oneshot_register("/dev/oneshot", oneshot);
      if (ret < 0)
        {
          _err("ERROR: Failed to register oneshot at /dev/oneshot: %d\n",
               ret);
        }
#endif
    }
#endif

#ifdef CONFIG_AJOYSTICK
  /* Initialize the simulated analog joystick input device */

  sim_ajoy_initialize();
#endif

#ifdef CONFIG_GRAPHICS_TRAVELER_ROMFSDEMO
  /* Special initialization for the Traveler game simulation */

  (void)trv_mount_world(0, CONFIG_GRAPHICS_TRAVELER_DEFPATH);
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, SIM_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount procfs at %s: %d\n",
           SIM_PROCFS_MOUNTPOINT, ret);
    }
#endif

  return OK;
}
