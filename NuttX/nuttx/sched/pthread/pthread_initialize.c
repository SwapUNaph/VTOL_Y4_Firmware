/****************************************************************************
 * sched/pthread/pthread_initialize.c
 *
 *   Copyright (C) 2007-2010, 2013 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_initialize
 *
 * Description:
 *   This is an internal OS function called only at power-up boot time.  It
 *   no longer does anything since all of the pthread data structures have
 *   been moved into the "task group"
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

void pthread_initialize(void)
{
}

/****************************************************************************
 * Name: pthread_takesemaphore and pthread_givesemaphore
 *
 * Description:
 *   Support managed access to the private data sets.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   0 on success or an ERROR on failure with errno value set to EINVAL.
 *   Note that the errno EINTR is never returned by this function.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_takesemaphore(sem_t *sem)
{
  /* Verify input parameters */

  if (sem)
    {
      /* Take the semaphore */

      while (sem_wait(sem) != OK)
        {
          /* Handle the special case where the semaphore wait was
           * awakened by the receipt of a signal.
           */

          if (get_errno() != EINTR)
            {
              set_errno(EINVAL);
              return ERROR;
            }
        }
      return OK;
    }
  else
    {
      /* NULL semaphore pointer! */

      set_errno(EINVAL);
      return ERROR;
    }
}

int pthread_givesemaphore(sem_t *sem)
{
  /* Verify input parameters */

  if (sem)
    {
      /* Give the semaphore */

      if (sem_post(sem) == OK)
        {
          return OK;
        }
      else
        {
          /* sem_post() reported an error */

          set_errno(EINVAL);
          return ERROR;
        }
    }
  else
    {
      /* NULL semaphore pointer! */

      set_errno(EINVAL);
      return ERROR;
    }
}

