/****************************************************************************
 * sched/pthread/pthread_mutextrylock.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_trylock
 *
 * Description:
 *   The function pthread_mutex_trylock() is identical to pthread_mutex_lock()
 *   except that if the mutex object referenced by mutex is currently locked
 *   (by any thread, including the current thread), the call returns immediately
 *   with the errno EBUSY.
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return from
 *   the signal handler the thread resumes waiting for the mutex as if it was
 *   not interrupted.
 *
 * Parameters:
 *   mutex - A reference to the mutex to be locked.
 *
 * Return Value:
 *   0 on success or an errno value on failure.  Note that the errno EINTR
 *   is never returned by pthread_mutex_trylock().
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   - This implementation does not return EAGAIN when the mutex could not be
 *     acquired because the maximum number of recursive locks for mutex has
 *     been exceeded.
 *
 ****************************************************************************/

int pthread_mutex_trylock(FAR pthread_mutex_t *mutex)
{
  int ret = OK;

  sinfo("mutex=0x%p\n", mutex);

  if (!mutex)
    {
      ret = EINVAL;
    }
  else
    {
      int mypid = (int)getpid();

      /* Make sure the semaphore is stable while we make the following
       * checks.  This all needs to be one atomic action.
       */

      sched_lock();

      /* Try to get the semaphore. */

      if (sem_trywait((FAR sem_t *)&mutex->sem) == OK)
        {
          /* If we successfully obtained the semaphore, then indicate
           * that we own it.
           */

          mutex->pid = mypid;

#ifdef CONFIG_MUTEX_TYPES
          if (mutex->type == PTHREAD_MUTEX_RECURSIVE)
            {
              mutex->nlocks = 1;
            }
#endif
        }

      /* Was it not available? */

      else if (get_errno() == EAGAIN)
        {
#ifdef CONFIG_MUTEX_TYPES

          /* Check if recursive mutex was locked by ourself. */

          if (mutex->type == PTHREAD_MUTEX_RECURSIVE && mutex->pid == mypid)
            {
              /* Increment the number of locks held and return successfully. */

              mutex->nlocks++;
            }
          else
            {
              ret = EBUSY;
            }
#else
          ret = EBUSY;
#endif
        }
      else
        {
          ret = EINVAL;
        }

      sched_unlock();
    }

  sinfo("Returning %d\n", ret);
  return ret;
}



