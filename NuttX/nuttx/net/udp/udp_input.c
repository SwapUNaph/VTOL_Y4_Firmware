/****************************************************************************
 * net/udp/udp_input.c
 * Handling incoming UDP input
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "utils/utils.h"
#include "udp/udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_input
 *
 * Description:
 *   Handle incoming UDP input
 *
 * Parameters:
 *   dev   - The device driver structure containing the received UDP packet
 *   udp   - A pointer to the UDP header in the packet
 *   iplen - Length of the IP and UDP headers
 *
 * Return:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

static int udp_input(FAR struct net_driver_s *dev, unsigned int iplen)
{
  FAR struct udp_hdr_s *udp;
  FAR struct udp_conn_s *conn;
  unsigned int udpiplen;
  unsigned int hdrlen;
#ifdef CONFIG_NET_UDP_CHECKSUMS
  uint16_t chksum;
#endif
  int ret = OK;

  /* Update the count of UDP packets received */

#ifdef CONFIG_NET_STATISTICS
  g_netstats.udp.recv++;
#endif

  /* Get a pointer to the UDP header.  The UDP header lies just after the
   * the link layer header and the IP header.
   */

  udp = (FAR struct udp_hdr_s *)&dev->d_buf[iplen + NET_LL_HDRLEN(dev)];

  /* Get the size of the IP header and the UDP header */

  udpiplen = iplen + UDP_HDRLEN;

  /* Get the size of the link layer header, the IP header, and the UDP header */

  hdrlen = udpiplen + NET_LL_HDRLEN(dev);

  /* UDP processing is really just a hack. We don't do anything to the UDP/IP
   * headers, but let the UDP application do all the hard work. If the
   * application sets d_sndlen, it has a packet to send.
   */

  dev->d_len    -= udpiplen;
  dev->d_appdata = &dev->d_buf[hdrlen];

#ifdef CONFIG_NET_UDP_CHECKSUMS
  chksum = udp->udpchksum;
  if (chksum != 0)
    {
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv6(dev->d_flags))
#endif
        {
          chksum = ~udp_ipv6_chksum(dev);
        }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      else
#endif
        {
          chksum = ~udp_ipv4_chksum(dev);
        }
#endif /* CONFIG_NET_IPv6 */
     }

   if (chksum != 0)
     {
#ifdef CONFIG_NET_STATISTICS
      g_netstats.udp.drop++;
      g_netstats.udp.chkerr++;
#endif
      nwarn("WARNING: Bad UDP checksum\n");
      dev->d_len = 0;
    }
  else
#endif
    {
      /* Demultiplex this UDP packet between the UDP "connections".
       *
       * REVISIT:  The logic here expects either a single receive socket or
       * none at all.  However, multiple sockets should be capable of
       * receiving a UDP datagram (multicast reception).  This could be
       * handled easily by something like:
       *
       *   for (conn = NULL; conn = udp_active(dev, udp); )
       *
       * If the callback logic that receives a packet responds with an
       * outgoing packet, then it will over-write the received buffer,
       * however.  recvfrom() will not do that, however.  We would have to
       * make that the rule: Recipients of a UDP packet must treat the
       * packet as read-only.
       */

      conn = udp_active(dev, udp);
      if (conn)
        {
          uint16_t flags;

          /* Set-up for the application callback */

          dev->d_appdata = &dev->d_buf[hdrlen];
          dev->d_sndlen  = 0;

          /* Perform the application callback */

          flags = udp_callback(dev, conn, UDP_NEWDATA);

          /* If the operation was successful, the UDP_NEWDATA flag is removed
           * and thus the packet can be deleted (OK will be returned).
           */

          if ((flags & UDP_NEWDATA) != 0)
            {
              /* No.. the packet was not processed now.  Return ERROR so
               * that the driver may retry again later.
               */

              ret = ERROR;
            }

          /* If the application has data to send, setup the UDP/IP header */

          if (dev->d_sndlen > 0)
            {
              udp_send(dev, conn);
            }
        }
      else
        {
          nwarn("WARNING: No listener on UDP port\n");
          dev->d_len = 0;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_ipv4_input
 *
 * Description:
 *   Handle incoming UDP input in an IPv4 packet
 *
 * Parameters:
 *   dev - The device driver structure containing the received UDP packet
 *
 * Return:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int udp_ipv4_input(FAR struct net_driver_s *dev)
{
  /* Configure to receive an UDP IPv4 packet */

  udp_ipv4_select(dev);

  /* Then process in the UDP IPv4 input */

  return udp_input(dev, IPv4_HDRLEN);
}
#endif

/****************************************************************************
 * Name: udp_ipv6_input
 *
 * Description:
 *   Handle incoming UDP input in an IPv6 packet
 *
 * Parameters:
 *   dev - The device driver structure containing the received UDP packet
 *
 * Return:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int udp_ipv6_input(FAR struct net_driver_s *dev)
{
  /* Configure to receive an UDP IPv6 packet */

  udp_ipv6_select(dev);

  /* Then process in the UDP IPv6 input */

  return udp_input(dev, IPv6_HDRLEN);
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_UDP */
