/****************************************************************************
 * sched/sched/sched_setpriority.c
 *
 *   Copyright (C) 2009, 2013, 2016 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_running_setpriority
 *
 * Description:
 *   This function sets the priority of a running task.  This does nothing
 *   if we are increasing the priority of a running task.  If we are dropping
 *   the priority of a running task, then this could cause then next lower
 *   priority task to run,
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to after all other tasks
 *   with the same priority.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

static inline void sched_running_setpriority(FAR struct tcb_s *tcb,
                                             int sched_priority)
{
  /* A context switch will occur if the new priority of the running
   * task becomes less than OR EQUAL TO the next highest priority
   * ready to run task.
   */

  if (sched_priority <= tcb->flink->sched_priority)
    {
      /* A context switch will occur. */

      up_reprioritize_rtr(tcb, (uint8_t)sched_priority);
    }

  /* Otherwise, we can just change priority since it has no effect */

  else
    {
      /* Change the task priority */

      tcb->sched_priority = (uint8_t)sched_priority;
    }
}

/****************************************************************************
 * Name:  sched_readytorun_setpriority
 *
 * Description:
 *   This function sets the priority of a ready-to-run task.  This may alter
 *   the position of the task in the ready-to-run list and if the priority
 *   is increased, may cause the task to become running.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

static void sched_readytorun_setpriority(FAR struct tcb_s *tcb,
                                         int sched_priority)
{
  FAR struct tcb_s *rtcb;

#ifdef CONFIG_SMP
  int cpu;

  /* CASE 2a. The task is ready-to-run (but not running) but not assigned to
   * a CPU. An increase in priority could cause a context switch may be caused
   * by the re-prioritization.  The task is not assigned and may run on any CPU.
   */

  if (tcb->task_state == TSTATE_TASK_READYTORUN)
    {
      cpu = sched_cpu_select(tcb->affinity);
    }

  /* CASE 2b.  The task is ready to run, and assigned to a CPU.  An increase
   * in priority could cause this task to become running but the task can
   * only run on its assigned CPU.
   */

  else
    {
      cpu = tcb->cpu;
    }

  /* The running task is the the task at the head of the g_assignedtasks[]
   * associated with the selected CPU.
   */

  rtcb = current_task(cpu);

#else
  /* CASE 2. The task is ready-to-run (but not running) and a context switch
   * may be caused by the re-prioritization.
   */

  rtcb = this_task();
#endif

  /* A context switch will occur if the new priority of the ready-to-run
   * task is (strictly) greater than the current running task
   */

  if (sched_priority > rtcb->sched_priority)
    {
      /* A context switch will occur. */

      up_reprioritize_rtr(tcb, (uint8_t)sched_priority);
    }

  /* Otherwise, we can just change priority and re-schedule (since it have
   * no other effect).
   */

  else
    {
      /* Remove the TCB from the ready-to-run task list that it resides in */

      ASSERT(!sched_removereadytorun(tcb));

      /* Change the task priority */

      tcb->sched_priority = (uint8_t)sched_priority;

      /* Put it back into the correct ready-to-run task list */

      ASSERT(!sched_addreadytorun(tcb));
    }
}

/****************************************************************************
 * Name:  sched_blocked_setpriority
 *
 * Description:
 *   Change the priority of a blocked tasks.  The only issue here is that
 *   the task may like in a prioritized or an non-prioritized queue.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

static inline void sched_blocked_setpriority(FAR struct tcb_s *tcb,
                                             int sched_priority)
{
  FAR dq_queue_t *tasklist;
  tstate_t task_state = tcb->task_state;

  /* CASE 3a. The task resides in a prioritized list. */

  tasklist = TLIST_BLOCKED(task_state);
  if (TLIST_ISPRIORITIZED(task_state))
    {
      /* Remove the TCB from the prioritized task list */

      dq_rem((FAR dq_entry_t *)tcb, tasklist);

      /* Change the task priority */

      tcb->sched_priority = (uint8_t)sched_priority;

      /* Put it back into the prioritized list at the correct position. */

      sched_addprioritized(tcb, tasklist);
    }

  /* CASE 3b. The task resides in a non-prioritized list. */

  else
    {
      /* Just change the task's priority */

      tcb->sched_priority = (uint8_t)sched_priority;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_setpriority
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to after all other tasks
 *   with the same priority.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   On success, sched_setparam() returns 0 (OK). On error, -1 (ERROR) is
 *   returned, and errno is set appropriately.
 *
 *  EINVAL The parameter 'param' is invalid or does not make sense for the
 *         current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setpriority(FAR struct tcb_s *tcb, int sched_priority)
{
  irqstate_t flags;

  /* Verify that the requested priority is in the valid range */

  if (sched_priority < SCHED_PRIORITY_MIN ||
      sched_priority > SCHED_PRIORITY_MAX)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* We need to assure that there there is no interrupt activity while
   * performing the following.
   */

  flags = enter_critical_section();

  /* There are three major cases (and two sub-cases) that must be considered: */

  switch (tcb->task_state)
    {
      /* CASE 1. The task is running and a context switch may be caused by
       * the re-prioritization
       */

      case TSTATE_TASK_RUNNING:
        sched_running_setpriority(tcb, sched_priority);
        break;

      /* CASE 2. The task is ready-to-run (but not running) and a context
       * switch may be caused by the re-prioritization
       */

      case TSTATE_TASK_READYTORUN:
#ifdef CONFIG_SMP
      case TSTATE_TASK_ASSIGNED:
#endif
        sched_readytorun_setpriority(tcb, sched_priority);
        break;


      /* CASE 3. The task is not in the ready to run list.  Changing its
       * Priority cannot effect the currently executing task.
       */

      default:
        sched_blocked_setpriority(tcb, sched_priority);
        break;
    }

  leave_critical_section(flags);
  return OK;
}
