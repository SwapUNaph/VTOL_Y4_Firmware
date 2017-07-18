/****************************************************************************
 * net/socket/net_dupsd.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "socket/socket.h"

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_dupsd
 *
 * Description:
 *   Clone a socket descriptor to an arbitrary descriptor number.  If file
 *   descriptors are implemented, then this is called by dup() for the case
 *   of socket file descriptors.  If file descriptors are not implemented,
 *   then this function IS dup().
 *
 ****************************************************************************/

int net_dupsd(int sockfd, int minsd)
{
  FAR struct socket *psock1;
  FAR struct socket *psock2;
  int sockfd2;
  int errcode;
  int ret;

  /* Make sure that the minimum socket descriptor is within the legal range.
   * The minimum value we receive is relative to file descriptor 0;  we need
   * map it relative of the first socket descriptor.
   */

#if CONFIG_NFILE_DESCRIPTORS > 0
  if (minsd >= CONFIG_NFILE_DESCRIPTORS)
    {
      minsd -= CONFIG_NFILE_DESCRIPTORS;
    }
  else
    {
      minsd = 0;
    }
#endif

  /* Lock the scheduler throughout the following */

  sched_lock();

  /* Get the socket structure underlying sockfd */

  psock1 = sockfd_socket(sockfd);

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock1 || psock1->s_crefs <= 0)
    {
      errcode = EBADF;
      goto errout;
    }

  /* Allocate a new socket descriptor */

  sockfd2 = sockfd_allocate(minsd);
  if (sockfd2 < 0)
    {
      errcode = ENFILE;
      goto errout;
    }

  /* Get the socket structure underlying the new descriptor */

  psock2 = sockfd_socket(sockfd2);
  if (!psock2)
    {
      errcode = ENOSYS; /* should not happen */
      goto errout;
    }

  /* Duplicate the socket state */

  ret = net_clone(psock1, psock2);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout;

    }

  sched_unlock();
  return sockfd2;

errout:
  sched_unlock();
  set_errno(errcode);
  return ERROR;
}

#endif /* defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0 */
