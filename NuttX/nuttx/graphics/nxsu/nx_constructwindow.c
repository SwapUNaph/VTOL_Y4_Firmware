/****************************************************************************
 * graphics/nxsu/nx_constructwindow.c
 *
 *   Copyright (C) 2008-2009, 2012-2013 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nx.h>

#include "nxfe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_constructwindow
 *
 * Description:
 *   This function is the same a nx_openwindow EXCEPT that the client provides
 *   the window structure instance.  nx_constructwindow will initialize the
 *   the pre-allocated window structure for use by NX.  This function is
 *   provided in addition to nx_open window in order to support a kind of
 *   inheritance:  The caller's window structure may include extensions that
 *   are not visible to NX.
 *
 *   NOTE:  hwnd must have been allocated using a user-space allocator that
 *   permits user access to the window.  Once provided to nx_constructwindow()
 *   that memory is owned and managed by NX.  On certain error conditions or
 *   when the window is closed, NX will free the window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   hwnd   - The pre-allocated window structure.
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately.  In the
 *   case of ERROR, NX will have deallocated the pre-allocated window.
 *
 ****************************************************************************/

int nx_constructwindow(NXHANDLE handle, NXWINDOW hwnd,
                       FAR const struct nx_callback_s *cb, FAR void *arg)
{
  FAR struct nxfe_state_s *fe = (FAR struct nxfe_state_s *)handle;
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  FAR struct nxbe_state_s *be = &fe->be;

#ifdef CONFIG_DEBUG_FEATURES
  if (!wnd)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (!fe || !cb)
    {
      kumm_free(wnd);
      errno = EINVAL;
      return ERROR;
    }
#endif

  /* Initialize the window structure */

  wnd->be           = be;
  wnd->cb           = cb;
  wnd->arg          = arg;

  /* Insert the new window at the top on the display.  topwnd is
   * never NULL (it may point only at the background window, however)
   */

  wnd->above        = NULL;
  wnd->below        = be->topwnd;

  be->topwnd->above = wnd;
  be->topwnd        = wnd;

  /* Report the initialize size/position of the window */

  nxfe_reportposition((NXWINDOW)wnd);

  /* Provide the initial mouse settings */

#ifdef CONFIG_NX_XYINPUT
  nxsu_mousereport(wnd);
#endif

  return OK;
}
