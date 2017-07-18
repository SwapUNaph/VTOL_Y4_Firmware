/****************************************************************************
 * sched/wqueue/work_process.c
 *
 *   Copyright (C) 2009-2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <queue.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use CLOCK_MONOTONIC if it is available.  CLOCK_REALTIME can cause bad
 * delays if the time is changed.
 */

#ifdef CONFIG_CLOCK_MONOTONIC
#  define WORK_CLOCK CLOCK_MONOTONIC
#else
#  define WORK_CLOCK CLOCK_REALTIME
#endif

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_process
 *
 * Description:
 *   This is the logic that performs actions placed on any work list.  This
 *   logic is the common underlying logic to all work queues.  This logic is
 *   part of the internal implementation of each work queue; it should not
 *   be called from application level logic.
 *
 * Input parameters:
 *   wqueue - Describes the work queue to be processed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_process(FAR struct kwork_wqueue_s *wqueue, systime_t period, int wndx)
{
  volatile FAR struct work_s *work;
  worker_t  worker;
  irqstate_t flags;
  FAR void *arg;
  systime_t elapsed;
  systime_t remaining;
  systime_t stick;
  systime_t ctick;
  systime_t next;

  /* Then process queued work.  We need to keep interrupts disabled while
   * we process items in the work list.
   */

  next  = period;
  flags = enter_critical_section();

  /* Get the time that we started this polling cycle in clock ticks. */

  stick = clock_systimer();

  /* And check each entry in the work queue.  Since we have disabled
   * interrupts we know:  (1) we will not be suspended unless we do
   * so ourselves, and (2) there will be no changes to the work queue
   */

  work = (FAR struct work_s *)wqueue->q.head;
  while (work)
    {
      /* Is this work ready?  It is ready if there is no delay or if
       * the delay has elapsed. qtime is the time that the work was added
       * to the work queue.  It will always be greater than or equal to
       * zero.  Therefore a delay of zero will always execute immediately.
       */

      ctick   = clock_systimer();
      elapsed = ctick - work->qtime;
      if (elapsed >= work->delay)
        {
          /* Remove the ready-to-execute work from the list */

          (void)dq_rem((struct dq_entry_s *)work, &wqueue->q);

          /* Extract the work description from the entry (in case the work
           * instance by the re-used after it has been de-queued).
           */

          worker = work->worker;

          /* Check for a race condition where the work may be nullified
           * before it is removed from the queue.
           */

          if (worker != NULL)
            {
              /* Extract the work argument (before re-enabling interrupts) */

              arg = work->arg;

              /* Mark the work as no longer being queued */

              work->worker = NULL;

              /* Do the work.  Re-enable interrupts while the work is being
               * performed... we don't have any idea how long this will take!
               */

              leave_critical_section(flags);
              worker(arg);

              /* Now, unfortunately, since we re-enabled interrupts we don't
               * know the state of the work list and we will have to start
               * back at the head of the list.
               */

              flags = enter_critical_section();
              work  = (FAR struct work_s *)wqueue->q.head;
            }
          else
            {
              /* Cancelled.. Just move to the next work in the list with
               * interrupts still disabled.
               */

              work = (FAR struct work_s *)work->dq.flink;
            }
        }
      else /* elapsed < work->delay */
        {
          /* This one is not ready.
           *
           * NOTE that elapsed is relative to the the current time,
           * not the time of beginning of this queue processing pass.
           * So it may need an adjustment.
           */

          elapsed += (ctick - stick);
          if (elapsed > work->delay)
            {
              /* The delay has expired while we are processing */

              elapsed = work->delay;
            }

          /* Will it be ready before the next scheduled wakeup interval? */

          remaining = work->delay - elapsed;
          if (remaining < next)
            {
              /* Yes.. Then schedule to wake up when the work is ready */

              next = remaining;
            }

          /* Then try the next in the list. */

          work = (FAR struct work_s *)work->dq.flink;
        }
    }

#if defined(CONFIG_SCHED_LPWORK) && CONFIG_SCHED_LPNTHREADS > 0
  /* Value of zero for period means that we should wait indefinitely until
   * signalled.  This option is used only for the case where there are
   * multiple, low-priority worker threads.  In that case, only one of
   * the threads does the poll... the others simple.  In all other cases
   * period will be non-zero and equal to wqueue->delay.
   */

  if (period == 0)
    {
      sigset_t set;

      /* Wait indefinitely until signalled with SIGWORK */

      sigemptyset(&set);
      sigaddset(&set, SIGWORK);

      wqueue->worker[wndx].busy = false;
      DEBUGVERIFY(sigwaitinfo(&set, NULL));
       wqueue->worker[wndx].busy = true;
    }
  else
#endif
    {
      /* Get the delay (in clock ticks) since we started the sampling */

      elapsed = clock_systimer() - stick;
      if (elapsed < period && next > 0)
        {
          /* How much time would we need to delay to get to the end of the
           * sampling period?  The amount of time we delay should be the smaller
           * of the time to the end of the sampling period and the time to the
           * next work expiry.
           */

          remaining = period - elapsed;
          next      = MIN(next, remaining);

          /* Wait awhile to check the work list.  We will wait here until
           * either the time elapses or until we are awakened by a signal.
           * Interrupts will be re-enabled while we wait.
           */

          wqueue->worker[wndx].busy = false;
          usleep(next * USEC_PER_TICK);
          wqueue->worker[wndx].busy = true;
        }
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_SCHED_WORKQUEUE */
