/****************************************************************************
 * net/igmp/igmp_group.c
 * IGMP group data structure management logic
 *
 *   Copyright (C) 2010, 2013-2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The NuttX implementation of IGMP was inspired by the IGMP add-on for the
 * lwIP TCP/IP stack by Steve Reynolds:
 *
 *   Copyright (c) 2002 CITEL Technologies Ltd.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdlib.h>
#include <string.h>
#include <queue.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_NET_IPv6
#  error "IGMP for IPv6 not supported"
#endif

#ifndef CONFIG_PREALLOC_IGMPGROUPS
#  define CONFIG_PREALLOC_IGMPGROUPS 4
#endif

/* Debug ********************************************************************/

#undef IGMP_GRPDEBUG /* Define to enable detailed IGMP group debug */

#ifndef CONFIG_NET_IGMP
#  undef IGMP_GRPDEBUG
#endif

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef IGMP_GRPDEBUG
#    define grperr(format, ...)    nerr(format, ##__VA_ARGS__)
#    define grpinfo(format, ...)   ninfo(format, ##__VA_ARGS__)
#  else
#    define grperr(x...)
#    define grpinfo(x...)
#  endif
#else
#  ifdef IGMP_GRPDEBUG
#    define grperr    nerr
#    define grpinfo   ninfo
#  else
#    define grperr    (void)
#    define grpinfo   (void)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* kmm_malloc() cannot be called from an interrupt handler.  To work around this,
 * a small number of IGMP groups are preallocated just for use in interrupt
 * handling logic.
 */

#if CONFIG_PREALLOC_IGMPGROUPS > 0
static struct igmp_group_s g_preallocgrps[CONFIG_PREALLOC_IGMPGROUPS];
static FAR sq_queue_t g_freelist;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_grpheapalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   Calls kmm_malloc and so cannot be called from an interrupt handler.
 *
 ****************************************************************************/

static inline FAR struct igmp_group_s *igmp_grpheapalloc(void)
{
  return (FAR struct igmp_group_s *)kmm_zalloc(sizeof(struct igmp_group_s));
}

/****************************************************************************
 * Name:  igmp_grpprealloc
 *
 * Description:
 *   Allocate a new group from the pre-allocated groups.
 *
 * Assumptions:
 *   This function should only be called from an interrupt handler (or with
 *   interrupts disabled).
 *
 ****************************************************************************/

#if CONFIG_PREALLOC_IGMPGROUPS > 0
static inline FAR struct igmp_group_s *igmp_grpprealloc(void)
{
  FAR struct igmp_group_s *group =
    (FAR struct igmp_group_s *)sq_remfirst(&g_freelist);

  if (group)
    {
      memset(group, 0, sizeof(struct igmp_group_s));
      group->flags = IGMP_PREALLOCATED;
    }

  return group;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_grpinit
 *
 * Description:
 *   One-time initialization of group data.
 *
 * Assumptions:
 *   Called only during early boot phases (pre-multitasking).
 *
 ****************************************************************************/

void igmp_grpinit(void)
{
  FAR struct igmp_group_s *group;
  int i;

  grpinfo("Initializing\n");

#if CONFIG_PREALLOC_IGMPGROUPS > 0
  for (i = 0; i < CONFIG_PREALLOC_IGMPGROUPS; i++)
    {
      group = &g_preallocgrps[i];
      sq_addfirst((FAR sq_entry_t *)group, &g_freelist);
    }
#endif
}

/****************************************************************************
 * Name:  igmp_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpalloc(FAR struct net_driver_s *dev,
                                       FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group;

  ninfo("addr: %08x dev: %p\n", *addr, dev);
  if (up_interrupt_context())
    {
#if CONFIG_PREALLOC_IGMPGROUPS > 0
      grpinfo("Use a pre-allocated group entry\n");
      group = igmp_grpprealloc();
#else
      grperr("ERROR: Cannot allocate from interrupt handler\n");
      group = NULL;
#endif
    }
  else
    {
      grpinfo("Allocate from the heap\n");
      group = igmp_grpheapalloc();
    }

  grpinfo("group: %p\n", group);

  /* Check if we successfully allocated a group structure */

  if (group)
    {
      /* Initialize the non-zero elements of the group structure */

      net_ipv4addr_copy(group->grpaddr, *addr);

      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      sem_init(&group->sem, 0, 0);
      sem_setprotocol(&group->sem, SEM_PRIO_NONE);

      /* Initialize the group timer (but don't start it yet) */

      group->wdog = wd_create();
      DEBUGASSERT(group->wdog);

      /* Interrupts must be disabled in order to modify the group list */

      net_lock();

      /* Add the group structure to the list in the device structure */

      sq_addfirst((FAR sq_entry_t *)group, &dev->grplist);
      net_unlock();
    }

  return group;
}

/****************************************************************************
 * Name:  igmp_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpfind(FAR struct net_driver_s *dev,
                                      FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group;

  grpinfo("Searching for addr %08x\n", (int)*addr);

  /* We must disable interrupts because we don't which context we were
   * called from.
   */

  net_lock();
  for (group = (FAR struct igmp_group_s *)dev->grplist.head;
       group;
       group = group->next)
    {
      grpinfo("Compare: %08x vs. %08x\n", group->grpaddr, *addr);
      if (net_ipv4addr_cmp(group->grpaddr, *addr))
        {
          grpinfo("Match!\n");
          break;
        }
    }

  net_unlock();
  return group;
}

/****************************************************************************
 * Name:  igmp_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpallocfind(FAR struct net_driver_s *dev,
                                           FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group = igmp_grpfind(dev, addr);

  grpinfo("group: %p addr: %08x\n", group, (int)*addr);
  if (!group)
    {
      group = igmp_grpalloc(dev, addr);
    }

  grpinfo("group: %p\n", group);
  return group;
}

/****************************************************************************
 * Name:  igmp_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

void igmp_grpfree(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group)
{
  grpinfo("Free: %p flags: %02x\n", group, group->flags);

  /* Cancel the wdog */

  net_lock();
  wd_cancel(group->wdog);

  /* Remove the group structure from the group list in the device structure */

  sq_rem((FAR sq_entry_t *)group, &dev->grplist);

  /* Destroy the wait semaphore */

  (void)sem_destroy(&group->sem);

  /* Destroy the wdog */

  wd_delete(group->wdog);

  /* Then release the group structure resources.  Check first if this is one
   * of the pre-allocated group structures that we will retain in a free list.
   */

#if CONFIG_PREALLOC_IGMPGROUPS > 0
  if (IS_PREALLOCATED(group->flags))
    {
      grpinfo("Put back on free list\n");
      sq_addlast((FAR sq_entry_t *)group, &g_freelist);
      net_unlock();
    }
  else
#endif
    {
      /* No.. deallocate the group structure.  Use sched_kfree() just in case
       * this function is executing within an interrupt handler.
       */

      net_unlock();
      grpinfo("Call sched_kfree()\n");
      sched_kfree(group);
    }
}

#endif /* CONFIG_NET_IGMP */
