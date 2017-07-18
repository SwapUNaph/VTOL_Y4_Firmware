/****************************************************************************
 * sched/clock/clock_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_RTC
#  include <nuttx/irq.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/time.h>

#include "clock/clock.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Standard time definitions (in units of seconds) */

#define SEC_PER_MIN  ((time_t)60)
#define SEC_PER_HOUR ((time_t)60 * SEC_PER_MIN)
#define SEC_PER_DAY  ((time_t)24 * SEC_PER_HOUR)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
#ifdef CONFIG_SYSTEM_TIME64
volatile uint64_t g_system_timer;
#else
volatile uint32_t g_system_timer;
#endif
#endif

struct timespec   g_basetime;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_basetime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
#if defined(CONFIG_RTC_DATETIME)
/* Initialize the system time using a broken out date/time structure */

static inline int clock_basetime(FAR struct timespec *tp)
{
  struct tm rtctime;
  int ret;

  /* Get the broken-out time from the date/time RTC. */

  ret = up_rtc_getdatetime(&rtctime);
  if (ret >= 0)
    {
      /* And use the broken-out time to initialize the system time */

      tp->tv_sec  = mktime(&rtctime);
      tp->tv_nsec = 0;
    }

  return ret;
}

#elif defined(CONFIG_RTC_HIRES)

/* Initialize the system time using a high-resolution structure */

static inline int clock_basetime(FAR struct timespec *tp)
{
  /* Get the complete time from the hi-res RTC. */

  return up_rtc_gettime(tp);
}

#else

/* Initialize the system time using seconds only */

static inline int clock_basetime(FAR struct timespec *tp)
{
  /* Get the seconds (only) from the lo-resolution RTC */

  tp->tv_sec  = up_rtc_time();
  tp->tv_nsec = 0;
  return OK;
}

#endif /* CONFIG_RTC_HIRES */
#else /* CONFIG_RTC */

static inline int clock_basetime(FAR struct timespec *tp)
{
  time_t jdn = 0;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(CONFIG_START_YEAR, CONFIG_START_MONTH - 1,
                           CONFIG_START_DAY);

  /* Set the base time as seconds into this julian day. */

  tp->tv_sec  = jdn * SEC_PER_DAY;
  tp->tv_nsec = 0;
  return OK;
}

#endif /* CONFIG_RTC */

/****************************************************************************
 * Name: clock_inittime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

static void clock_inittime(void)
{
  /* (Re-)initialize the time value to match the RTC */

#ifndef CONFIG_CLOCK_TIMEKEEPING
#ifndef CONFIG_RTC_HIRES
  clock_basetime(&g_basetime);
#endif
#ifndef CONFIG_SCHED_TICKLESS
  g_system_timer = 0;
#endif
#else
  clock_inittimekeeping();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_initialize
 *
 * Description:
 *   Perform one-time initialization of the timing facilities.
 *
 ****************************************************************************/

void clock_initialize(void)
{
#if defined(CONFIG_RTC) && !defined(CONFIG_RTC_EXTERNAL)
  /* Initialize the internal RTC hardware.  Initialization of external RTC
   * must be deferred until the system has booted.
   */

  up_rtc_initialize();
#endif

  /* Initialize the time value to match the RTC */

  clock_inittime();
}

/****************************************************************************
 * Name: clock_synchronize
 *
 * Description:
 *   Synchronize the system timer to a hardware RTC.  This operation is
 *   normally performed automatically by the system during clock
 *   initialization.  However, the user may also need to explicitly re-
 *   synchronize the system timer to the RTC under certain conditions where
 *   the system timer is known to be in error.  For example, in certain low-
 *   power states, the system timer may be stopped but the RTC will continue
 *   keep correct time.  After recovering from such low-power state, this
 *   function should be called to restore the correct system time.
 *
 *   Calling this function could result in system time going "backward" in
 *   time, especially with certain lower resolution RTC implementations.
 *   Time going backward could have bad consequences if there are ongoing
 *   timers and delays.  So use this interface with care.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
void clock_synchronize(void)
{
  irqstate_t flags;

  /* Re-initialize the time value to match the RTC */

  flags = enter_critical_section();
  clock_inittime();
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: clock_timer
 *
 * Description:
 *   This function must be called once every time the real time clock
 *   interrupt occurs.  The interval of this clock interrupt must be
 *   USEC_PER_TICK
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
void clock_timer(void)
{
  /* Increment the per-tick system counter */

  g_system_timer++;
}
#endif
