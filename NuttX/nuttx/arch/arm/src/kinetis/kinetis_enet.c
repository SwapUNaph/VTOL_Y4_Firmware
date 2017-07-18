/****************************************************************************
 * arch/arm/src/kinetis/kinetis_enet.c
 *
 *   Copyright (C) 2011-2012, 2014-2016 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_KINETIS_ENET)

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "up_arch.h"
#include "chip.h"
#include "kinetis.h"
#include "kinetis_config.h"
#include "chip/kinetis_pinmux.h"
#include "chip/kinetis_sim.h"
#include "chip/kinetis_mpu.h"
#include "chip/kinetis_enet.h"

#if defined(KINETIS_NENET) && KINETIS_NENET > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

  /* Select work queue */

#  if defined(CONFIG_KINETIS_EMAC_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_KINETIS_EMAC_LPWORK)
#    define ETHWORK LPWORK
#  else
#    error Neither CONFIG_KINETIS_EMAC_HPWORK nor CONFIG_KINETIS_EMAC_LPWORK defined
#  endif
#endif

/* CONFIG_KINETIS_ENETNETHIFS determines the number of physical interfaces
 * that will be supported.
 */

#if CONFIG_KINETIS_ENETNETHIFS != 1
#  error "CONFIG_KINETIS_ENETNETHIFS must be one for now"
#endif

#if CONFIG_KINETIS_ENETNTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_KINETIS_ENETNRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif

#define NENET_NBUFFERS \
  (CONFIG_KINETIS_ENETNTXBUFFERS+CONFIG_KINETIS_ENETNRXBUFFERS)

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second.
 */

#define KINETIS_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define KINETIS_TXTIMEOUT (60*CLK_TCK)
#define MII_MAXPOLLS      (0x1ffff)
#define LINK_WAITUS       (500*1000)

/* PHY definitions.
 *
 * The selected PHY must be selected from the drivers/net/Kconfig PHY menu.
 * A description of the PHY must be provided here.  That description must
 * include:
 *
 * 1. BOARD_PHY_NAME: A PHY name string (for debug output),
 * 2. BOARD_PHYID1 and BOARD_PHYID2: The PHYID1 and PHYID2 values (from
 *    include/nuttx/net/mii.h)
 * 3. BOARD_PHY_STATUS:  The address of the status register to use when
 *    querying link status (from include/nuttx/net/mii.h)
 * 4. BOARD_PHY_ISDUPLEX:  A macro that can convert the status register
 *    value into a boolean: true=duplex mode, false=half-duplex mode
 * 5. BOARD_PHY_10BASET:  A macro that can convert the status register
 *    value into a boolean: true=10Base-T, false=Not 10Base-T
 * 6. BOARD_PHY_100BASET:  A macro that can convert the status register
 *    value into a boolean: true=100Base-T, false=Not 100Base-T
 *
 * The Tower SER board uses a KSZ8041 PHY.
 * The Freedom K64F board uses a KSZ8081 PHY
 */

#if defined(CONFIG_ETH0_PHY_KSZ8041)
#  define BOARD_PHY_NAME        "KSZ8041"
#  define BOARD_PHYID1          MII_PHYID1_KSZ8041
#  define BOARD_PHYID2          MII_PHYID2_KSZ8041
#  define BOARD_PHY_STATUS      MII_KSZ8041_PHYCTRL2
#  define BOARD_PHY_ISDUPLEX(s) (((s) & (4 << MII_PHYCTRL2_MODE_SHIFT)) != 0)
#  define BOARD_PHY_10BASET(s)  (((s) & (1 << MII_PHYCTRL2_MODE_SHIFT)) != 0)
#  define BOARD_PHY_100BASET(s) (((s) & (2 << MII_PHYCTRL2_MODE_SHIFT)) != 0)
#elif defined(CONFIG_ETH0_PHY_KSZ8081)
#  define BOARD_PHY_NAME        "KSZ8081"
#  define BOARD_PHYID1          MII_PHYID1_KSZ8081
#  define BOARD_PHYID2          MII_PHYID2_KSZ8081
#  define BOARD_PHY_STATUS      MII_KSZ8081_PHYCTRL2
#  define BOARD_PHY_ISDUPLEX(s) (((s) & (4 << MII_PHYCTRL2_MODE_SHIFT)) != 0)
#  define BOARD_PHY_10BASET(s)  (((s) & (1 << MII_PHYCTRL2_MODE_SHIFT)) != 0)
#  define BOARD_PHY_100BASET(s) (((s) & (2 << MII_PHYCTRL2_MODE_SHIFT)) != 0)
#else
#  error "Unrecognized or missing PHY selection"
#endif

/* Estimate the hold time to use based on the peripheral (bus) clock:
 *
 *   HOLD_TIME = (2*BUS_FREQ_MHZ)/5 + 1
 *             = (BUS_FREQ)/2500000 + 1
 *
 * For example, if BUS_FREQ_MHZ=48 (MHz):
 *
 *   HOLD_TIME = 48Mhz, hold time clocks
 *             = 48000000/2500000 + 1
 *             = 20
 */

#define KINETIS_MII_SPEED  (BOARD_BUS_FREQ/2500000 + 1)
#if KINETIS_MII_SPEED > 63
#  error "KINETIS_MII_SPEED is out-of-range"
#endif

/* Interrupt groups */

#define RX_INTERRUPTS     (ENET_INT_RXF | ENET_INT_RXB)
#define TX_INTERRUPTS      ENET_INT_TXF
#define ERROR_INTERRUPTS  (ENET_INT_UN    | ENET_INT_RL   | ENET_INT_LC | \
                           ENET_INT_EBERR | ENET_INT_BABT | ENET_INT_BABR)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

#define KINETIS_BUF_SIZE  ((CONFIG_NET_ETH_MTU & 0xfffffff0) + 0x10)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* The kinetis_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct kinetis_driver_s
{
  bool bifup;                  /* true:ifup false:ifdown */
  uint8_t txtail;              /* The oldest busy TX descriptor */
  uint8_t txhead;              /* The next TX descriptor to use */
  uint8_t rxtail;              /* The next RX descriptor to use */
  uint8_t phyaddr;             /* Selected PHY address */
  WDOG_ID txpoll;              /* TX poll timer */
  WDOG_ID txtimeout;           /* TX timeout timer */
  struct work_s work;          /* For deferring work to the work queue */
  struct enet_desc_s *txdesc;  /* A pointer to the list of TX descriptor */
  struct enet_desc_s *rxdesc;  /* A pointer to the list of RX descriptors */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;     /* Interface understood by the network */

  /* The DMA descriptors.  A unaligned uint8_t is used to allocate the
   * memory; 16 is added to assure that we can meet the descriptor alignment
   * requirements.
   */

  uint8_t desc[NENET_NBUFFERS * sizeof(struct enet_desc_s) + 16];

  /* The DMA buffers.  Again, A unaligned uint8_t is used to allocate the
   * memory; 16 is added to assure that we can meet the descriptor alignment
   * requirements.
   */

  uint8_t buffers[NENET_NBUFFERS * KINETIS_BUF_SIZE + 16];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct kinetis_driver_s g_enet[CONFIG_KINETIS_ENETNETHIFS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Utility functions */

#ifndef KINETIS_BUFFERS_SWAP
#  define kinesis_swap32(value) (value)
#  define kinesis_swap16(value) (value)
#else
#if 0 /* Use builtins if the compiler supports them */
static inline uint32_t kinesis_swap32(uint32_t value);
static inline uint16_t kinesis_swap16(uint16_t value);
#else
#  define kinesis_swap32 __builtin_bswap32
#  define kinesis_swap16 __builtin_bswap16
#endif
#endif

/* Common TX logic */

static bool kinetics_txringfull(FAR struct kinetis_driver_s *priv);
static int  kinetis_transmit(FAR struct kinetis_driver_s *priv);
static int  kinetis_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void kinetis_receive(FAR struct kinetis_driver_s *priv);
static void kinetis_txdone(FAR struct kinetis_driver_s *priv);

static void kinetis_interrupt_work(FAR void *arg);
static int  kinetis_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void kinetis_txtimeout_work(FAR void *arg);
static void kinetis_txtimeout_expiry(int argc, uint32_t arg, ...);

static void kinetis_poll_work(FAR void *arg);
static void kinetis_polltimer_expiry(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int kinetis_ifup(struct net_driver_s *dev);
static int kinetis_ifdown(struct net_driver_s *dev);

static void kinetis_txavail_work(FAR void *arg);
static int kinetis_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_IGMP
static int kinetis_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
static int kinetis_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_PHY_IOCTL
static int kinetis_ioctl(struct net_driver_s *dev, int cmd, long arg);
#endif

/* PHY/MII support */

static inline void kinetis_initmii(struct kinetis_driver_s *priv);
static int kinetis_writemii(struct kinetis_driver_s *priv, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t data);
static int kinetis_readmii(struct kinetis_driver_s *priv, uint8_t phyaddr,
                           uint8_t regaddr, uint16_t *data);
static inline int kinetis_initphy(struct kinetis_driver_s *priv);

/* Initialization */

static void kinetis_initbuffers(struct kinetis_driver_s *priv);
static void kinetis_reset(struct kinetis_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: kinetis_swap16/32
 *
 * Description:
 *   The descriptors are represented by structures  Unfortunately, when the
 *   structures are overlayed on the data, the bytes are reversed because
 *   the underlying hardware writes the data in big-endian byte order.
 *
 * Parameters:
 *   value  - The value to be byte swapped
 *
 * Returned Value:
 *   The byte swapped value
 *
 ****************************************************************************/

#if 0 /* Use builtins if the compiler supports them */
#ifndef CONFIG_ENDIAN_BIG
static inline uint32_t kinesis_swap32(uint32_t value)
{
  uint32_t result = 0;

  __asm__ __volatile__
  (
    "rev %0, %1"
    :"=r" (result)
    : "r"(value)
  );
  return result;
}

static inline uint16_t kinesis_swap16(uint16_t value)
{
  uint16_t result = 0;

  __asm__ __volatile__
  (
    "revsh %0, %1"
    :"=r" (result)
    : "r"(value)
  );
  return result;
}
#endif
#endif

/****************************************************************************
 * Function: kinetics_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool kinetics_txringfull(FAR struct kinetis_driver_s *priv)
{
  uint8_t txnext;

  /* Check if there is room in the hardware to hold another outgoing
   * packet.  The ring is full if incrementing the head pointer would
   * collide with the tail pointer.
   */

  txnext = priv->txhead + 1;
  if (txnext >= CONFIG_KINETIS_ENETNTXBUFFERS)
    {
      txnext = 0;
    }

  return priv->txtail == txnext;
}

/****************************************************************************
 * Function: kinetis_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int kinetis_transmit(FAR struct kinetis_driver_s *priv)
{
  struct enet_desc_s *txdesc;
  uint32_t regval;
  uint8_t *buf;

  /* Since this can be called from kinetis_receive, it is possible that
   * the transmit queue is full, so check for that now.  If this is the
   * case, the outgoing packet will be dropped (e.g. an ARP reply)
   */

  if (kinetics_txringfull(priv))
    {
      return -EBUSY;
    }

  /* When we get here the TX descriptor should show that the previous
   * transfer has  completed.  If we get here, then we are committed to
   * sending a packet; Higher level logic must have assured that there is
   * no transmission in progress.
   */

  txdesc = &priv->txdesc[priv->txhead];
  priv->txhead++;
  if (priv->txhead >= CONFIG_KINETIS_ENETNTXBUFFERS)
    {
      priv->txhead = 0;
    }

  DEBUGASSERT(priv->txtail != priv->txhead &&
             (txdesc->status1 & TXDESC_R) == 0);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  /* Setup the buffer descriptor for transmission: address=priv->dev.d_buf,
   * length=priv->dev.d_len
   */

  txdesc->length  = kinesis_swap16(priv->dev.d_len);
#ifdef CONFIG_KINETIS_ENETENHANCEDBD
  txdesc->bdu     = 0x00000000;
  txdesc->status2 = TXDESC_INT | TXDESC_TS; /* | TXDESC_IINS | TXDESC_PINS; */
#endif
  txdesc->status1 |= (TXDESC_R | TXDESC_L | TXDESC_TC);

  buf = (uint8_t*)kinesis_swap32((uint32_t)priv->dev.d_buf);
  if (priv->rxdesc[priv->rxtail].data == buf)
    {
       struct enet_desc_s *rxdesc = &priv->rxdesc[priv->rxtail];

       /* Data was written into the RX buffer, so swap the TX and RX buffers */

       DEBUGASSERT((rxdesc->status1 & RXDESC_E) == 0);
       rxdesc->data = txdesc->data;
       txdesc->data = buf;
    }
  else
    {
       ASSERT(txdesc->data == buf);
    }

  /* Start the TX transfer (if it was not already waiting for buffers) */

  putreg32(ENET_TDAR, KINETIS_ENET_TDAR);

  /* Enable TX interrupts */

  regval  = getreg32(KINETIS_ENET_EIMR);
  regval |= TX_INTERRUPTS;
  putreg32(regval, KINETIS_ENET_EIMR);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, KINETIS_TXTIMEOUT, kinetis_txtimeout_expiry, 1,
                 (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: kinetis_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing packets ready
 *   to send.  This is a callback from devif_poll().  devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int kinetis_txpoll(struct net_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
        {
          arp_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv6 */

      /* Send the packet */

      kinetis_transmit(priv);
      priv->dev.d_buf =
        (uint8_t*)kinesis_swap32((uint32_t)priv->txdesc[priv->txhead].data);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */

      if (kinetics_txringfull(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: kinetis_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void kinetis_receive(FAR struct kinetis_driver_s *priv)
{
  /* Loop while there are received packets to be processed */

  while ((priv->rxdesc[priv->rxtail].status1 & RXDESC_E) == 0)
    {
      /* Update statistics */

      NETDEV_RXPACKETS(&priv->dev);

      /* Copy the buffer pointer to priv->dev.d_buf.  Set amount of data in
       * priv->dev.d_len
       */

      priv->dev.d_len = kinesis_swap16(priv->rxdesc[priv->rxtail].length);
      priv->dev.d_buf =
        (uint8_t *)kinesis_swap32((uint32_t)priv->rxdesc[priv->rxtail].data);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

       pkt_input(&priv->dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->dev);

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
                {
                  arp_out(&priv->dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&priv->dev);
                }
#endif

              /* And send the packet */

              kinetis_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("Iv6 frame\n");
          NETDEV_RXIPV6(&priv->dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->dev.d_flags))
                {
                  arp_out(&priv->dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->dev);
                }
#endif

              /* And send the packet */

              kinetis_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          NETDEV_RXARP(&priv->dev);
          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              kinetis_transmit(priv);
            }
        }
#endif
      else
        {
          NETDEV_RXDROPPED(&priv->dev);
        }

      /* Point the packet buffer back to the next TX buffer, which will be used during
       * the next write.  If the write queue is full, then this will point at an active
       * buffer, which must not be written to.  This is OK because devif_poll won't be
       * called unless the queue is not full.
       */

      priv->dev.d_buf =
        (uint8_t*)kinesis_swap32((uint32_t)priv->txdesc[priv->txhead].data);
      priv->rxdesc[priv->rxtail].status1 |= RXDESC_E;

      /* Update the index to the next descriptor */

      priv->rxtail++;
      if (priv->rxtail >= CONFIG_KINETIS_ENETNRXBUFFERS)
        {
          priv->rxtail = 0;
        }

      /* Indicate that there have been empty receive buffers produced */

      putreg32(ENET_RDAR, KINETIS_ENET_RDAR);
    }
}

/****************************************************************************
 * Function: kinetis_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void kinetis_txdone(FAR struct kinetis_driver_s *priv)
{
  uint32_t regval;

  /* Verify that the oldest descriptor descriptor completed */

  while (((priv->txdesc[priv->txtail].status1 & TXDESC_R) == 0) &&
         (priv->txtail != priv->txhead))
    {
      /* Yes.. bump up the tail pointer, making space for a new TX descriptor */

      priv->txtail++;
      if (priv->txtail >= CONFIG_KINETIS_ENETNTXBUFFERS)
        {
          priv->txtail = 0;
        }

      /* Update statistics */

      NETDEV_TXDONE(&priv->dev);
    }

  /* Are there other transmissions queued? */

  if (priv->txtail == priv->txhead)
    {
      /* No.. Cancel the TX timeout and disable further Tx interrupts. */

      wd_cancel(priv->txtimeout);

      regval  = getreg32(KINETIS_ENET_EIMR);
      regval &= ~TX_INTERRUPTS;
      putreg32(regval, KINETIS_ENET_EIMR);
    }

  /* There should be space for a new TX in any event.  Poll the network for new XMIT
   * data
   */

  (void)devif_poll(&priv->dev, kinetis_txpoll);
}

/****************************************************************************
 * Function: kinetis_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void kinetis_interrupt_work(FAR void *arg)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;
  uint32_t pending;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get the set of unmasked, pending interrupt. */

  pending = getreg32(KINETIS_ENET_EIR) & getreg32(KINETIS_ENET_EIMR);

  /* Clear the pending interrupts */

  putreg32(pending, KINETIS_ENET_EIR);

  /* Check for the receipt of a packet */

  if ((pending & ENET_INT_RXF) != 0)
    {
      /* A packet has been received, call kinetis_receive() to handle the
       * packet.
       */

      kinetis_receive(priv);
    }

  /* Check if a packet transmission has completed */

  if ((pending & ENET_INT_TXF) != 0)
    {
      /* Call kinetis_txdone to handle the end of transfer even.  NOTE that
       * this may disable further Tx interrupts if there are no pending
       * tansmissions.
       */

      kinetis_txdone(priv);
    }

  /* Check for errors */

  if (pending & ERROR_INTERRUPTS)
    {
      /* An error has occurred, update statistics */

      NETDEV_ERRORS(&priv->dev);

      /* Reinitialize all buffers. */

      kinetis_initbuffers(priv);

      /* Indicate that there have been empty receive buffers produced */

      putreg32(ENET_RDAR, KINETIS_ENET_RDAR);
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

#if 0
  up_enable_irq(KINETIS_IRQ_EMACTMR);
#endif
  up_enable_irq(KINETIS_IRQ_EMACTX);
  up_enable_irq(KINETIS_IRQ_EMACRX);
  up_enable_irq(KINETIS_IRQ_EMACMISC);
}

/****************************************************************************
 * Function: kinetis_interrupt
 *
 * Description:
 *   Three interrupt sources will vector this this function:
 *   1. Ethernet MAC transmit interrupt handler
 *   2. Ethernet MAC receive interrupt handler
 *   3.
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int kinetis_interrupt(int irq, FAR void *context)
{
  register FAR struct kinetis_driver_s *priv = &g_enet[0];

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(KINETIS_IRQ_EMACTMR);
  up_disable_irq(KINETIS_IRQ_EMACTX);
  up_disable_irq(KINETIS_IRQ_EMACRX);
  up_disable_irq(KINETIS_IRQ_EMACMISC);

  /* TODO: Determine if a TX transfer just completed */

    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be do race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(priv->txtimeout);
    }

  /* Cancel any pending poll work */

  work_cancel(ETHWORK, &priv->work);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->work, kinetis_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: kinetis_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void kinetis_txtimeout_work(FAR void *arg)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();
  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Take the interface down and bring it back up.  The is the most agressive
   * hardware reset.
   */

  (void)kinetis_ifdown(&priv->dev);
  (void)kinetis_ifup(&priv->dev);

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->dev, kinetis_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: kinetis_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void kinetis_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(KINETIS_IRQ_EMACTMR);
  up_disable_irq(KINETIS_IRQ_EMACTX);
  up_disable_irq(KINETIS_IRQ_EMACRX);
  up_disable_irq(KINETIS_IRQ_EMACMISC);

  /* Cancel any pending poll or interrupt work.  This will have no effect
   * on work that has already been started.
   */

  work_cancel(ETHWORK, &priv->work);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->work, kinetis_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: kinetis_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void kinetis_poll_work(FAR void *arg)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Check if there is there is a transmission in progress.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  net_lock();
  if (!kinetics_txringfull(priv))
    {
      /* If so, update TCP timing states and poll the network for new XMIT data. Hmmm..
       * might be bug here.  Does this mean if there is a transmit in progress,
       * we will missing TCP time state updates?
       */

      (void)devif_timer(&priv->dev, kinetis_txpoll);
    }

  /* Setup the watchdog poll timer again in any case */

  (void)wd_start(priv->txpoll, KINETIS_WDDELAY, kinetis_polltimer_expiry,
                 1, (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Function: kinetis_polltimer_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void kinetis_polltimer_expiry(int argc, uint32_t arg, ...)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions.
   */

  if (work_available(&priv->work))
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(ETHWORK, &priv->work, kinetis_poll_work, priv, 0);
    }
  else
    {
      /* No.. Just re-start the watchdog poll timer, missing one polling
       * cycle.
       */

      (void)wd_start(priv->txpoll, KINETIS_WDDELAY, kinetis_polltimer_expiry,
                     1, (wdparm_t)arg);
    }
}

/****************************************************************************
 * Function: kinetis_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int kinetis_ifup(struct net_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;
  uint8_t *mac = dev->d_mac.ether_addr_octet;
  uint32_t regval;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Initialize ENET buffers */

  kinetis_initbuffers(priv);

  /* Reset and disable the interface */

  kinetis_reset(priv);

  /* Configure the MII interface */

  kinetis_initmii(priv);

  /* Set the MAC address */

  putreg32((mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3],
           KINETIS_ENET_PALR);
  putreg32((mac[4] << 24) | (mac[5] << 16), KINETIS_ENET_PAUR);

  /* Clear the Individual and Group Address Hash registers */

  putreg32(0, KINETIS_ENET_IALR);
  putreg32(0, KINETIS_ENET_IAUR);
  putreg32(0, KINETIS_ENET_GALR);
  putreg32(0, KINETIS_ENET_GAUR);

  /* Configure the PHY */

  ret = kinetis_initphy(priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to configure the PHY: %d\n", ret);
      return ret;
    }

  /* Handle promiscuous mode */

#ifdef CONFIG_NET_PROMISCUOUS
  regval = getreg32(KINETIS_ENET_RCR);
  regval |= ENET_RCR_PROM;
  putreg32(regval, KINETIS_ENET_RCR);
#endif

  /* Select legacy of enhanced buffer descriptor format */

#ifdef CONFIG_KINETIS_ENETENHANCEDBD
  putreg32(ENET_ECR_EN1588, KINETIS_ENET_ECR);
#else
  putreg32(0, KINETIS_ENET_ECR);
#endif

  /* Set the RX buffer size */

  putreg32(KINETIS_BUF_SIZE, KINETIS_ENET_MRBR);

  /* Point to the start of the circular RX buffer descriptor queue */

  putreg32((uint32_t)priv->rxdesc, KINETIS_ENET_RDSR);

  /* Point to the start of the circular TX buffer descriptor queue */

  putreg32((uint32_t)priv->txdesc, KINETIS_ENET_TDSR);

  /* And enable the MAC itself */

  regval = getreg32(KINETIS_ENET_ECR);
  regval |= ENET_ECR_ETHEREN
#ifdef KINETIS_USE_DBSWAP
        | ENET_ECR_DBSWP
#endif
        ;
  putreg32(regval, KINETIS_ENET_ECR);

  /* Indicate that there have been empty receive buffers produced */

  putreg32(ENET_RDAR, KINETIS_ENET_RDAR);

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, KINETIS_WDDELAY, kinetis_polltimer_expiry, 1,
                 (wdparm_t)priv);

  /* Clear all pending ENET interrupt */

  putreg32(0xffffffff, KINETIS_ENET_EIR);

  /* Enable RX and error interrupts at the controller (TX interrupts are
   * still disabled).
   */

  putreg32(RX_INTERRUPTS | ERROR_INTERRUPTS,
           KINETIS_ENET_EIMR);

  /* Mark the interrupt "up" and enable interrupts at the NVIC */

  priv->bifup = true;
#if 0
  up_enable_irq(KINETIS_IRQ_EMACTMR);
#endif
  up_enable_irq(KINETIS_IRQ_EMACTX);
  up_enable_irq(KINETIS_IRQ_EMACRX);
  up_enable_irq(KINETIS_IRQ_EMACMISC);
  return OK;
}

/****************************************************************************
 * Function: kinetis_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int kinetis_ifdown(struct net_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupts at the NVIC */

  flags = enter_critical_section();
  up_disable_irq(KINETIS_IRQ_EMACTMR);
  up_disable_irq(KINETIS_IRQ_EMACTX);
  up_disable_irq(KINETIS_IRQ_EMACRX);
  up_disable_irq(KINETIS_IRQ_EMACMISC);
  putreg32(0, KINETIS_ENET_EIMR);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the kinetis_ifup() always
   * successfully brings the interface back up.
   */

  kinetis_reset(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: kinetis_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void kinetis_txavail_work(FAR void *arg)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!kinetics_txringfull(priv))
        {
          /* No, there is space for another transfer.  Poll the network for new
           * XMIT data.
           */

          (void)devif_poll(&priv->dev, kinetis_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: kinetis_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int kinetis_txavail(struct net_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->work))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->work, kinetis_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: kinetis_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int kinetis_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: kinetis_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int kinetis_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif

/****************************************************************************
 * Function: kinetis_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_PHY_IOCTL
static int kinetis_ioctl(struct net_driver_s *dev, int cmd, long arg)
{
  int ret;
  FAR struct kinetis_driver_s *priv =
    (FAR struct kinetis_driver_s *)dev->d_private;

  switch (cmd)
  {
  case SIOCGMIIPHY: /* Get MII PHY address */
    {
      struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
      req->phy_id = priv->phyaddr;
      ret = OK;
    }
    break;

  case SIOCGMIIREG: /* Get register from MII PHY */
    {
      struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
      ret = kinetis_readmii(priv, req->phy_id, req->reg_num, &req->val_out);
    }
    break;

  case SIOCSMIIREG: /* Set register in MII PHY */
    {
      struct mii_ioctl_data_s *req =
        (struct mii_ioctl_data_s *)((uintptr_t)arg);
      ret = kinetis_writemii(priv, req->phy_id, req->reg_num, req->val_in);
    }
    break;

  default:
    ret = -ENOTTY;
    break;
  }

  return ret;
}
#endif /* CONFIG_NETDEV_PHY_IOCTL */

/****************************************************************************
 * Function: kinetis_initmii
 *
 * Description:
 *   Configure the MII interface
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void kinetis_initmii(struct kinetis_driver_s *priv)
{
  /* Speed is based on the peripheral (bus) clock; hold time is 1 module
   * clock.  This hold time value may need to be increased on some platforms
   */

  putreg32(ENET_MSCR_HOLDTIME_1CYCLE |
           KINETIS_MII_SPEED << ENET_MSCR_MII_SPEED_SHIFT,
           KINETIS_ENET_MSCR);
}

/****************************************************************************
 * Function: kinetis_writemii
 *
 * Description:
 *   Write a 16-bit value to a PHY register.
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *   phyaddr - The PHY address
 *   regaddr - The PHY register address
 *   data    - The data to write to the PHY register
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int kinetis_writemii(struct kinetis_driver_s *priv, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);

  /* Initiatate the MII Management write */

  putreg32(data |
           2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_WRMII |
           1 << ENET_MMFR_ST_SHIFT,
           KINETIS_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(KINETIS_ENET_EIR) & ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Check for a timeout */

  if (timeout == MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);
  return OK;
}

/****************************************************************************
 * Function: kinetis_writemii
 *
 * Description:
 *   Read a 16-bit value from a PHY register.
 *
 * Parameters:
 *   priv    - Reference to the private ENET driver state structure
 *   phyaddr - The PHY address
 *   regaddr - The PHY register address
 *   data    - A pointer to the location to return the data
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int kinetis_readmii(struct kinetis_driver_s *priv, uint8_t phyaddr,
                           uint8_t regaddr, uint16_t *data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);

  /* Initiatate the MII Management read */

  putreg32(2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_RA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_RDMII |
           1 << ENET_MMFR_ST_SHIFT,
           KINETIS_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(KINETIS_ENET_EIR) & ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Check for a timeout */

  if (timeout >= MII_MAXPOLLS)
    {
      nerr("ERROR: Timed out waiting for transfer to complete\n");
      return -ETIMEDOUT;
    }

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);

  /* And return the MII data */

  *data = (uint16_t)(getreg32(KINETIS_ENET_MMFR) & ENET_MMFR_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: kinetis_initphy
 *
 * Description:
 *   Configure the PHY
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure;
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline int kinetis_initphy(struct kinetis_driver_s *priv)
{
  uint32_t rcr;
  uint32_t tcr;
  uint16_t phydata;
  uint8_t phyaddr;
  int retries;
  int ret;

  /* Loop (potentially infinitely?) until we successfully communicate with
   * the PHY.
   */

  for (phyaddr = 0; phyaddr < 32; phyaddr++)
    {
      ninfo("%s: Try phyaddr: %u\n", BOARD_PHY_NAME, phyaddr);

      /* Try to read PHYID1 few times using this address */

      retries = 0;
      do
        {
          usleep(LINK_WAITUS);
          ninfo("%s: Read PHYID1, retries=%d\n",  BOARD_PHY_NAME, retries + 1);
          phydata = 0xffff;
          ret = kinetis_readmii(priv, phyaddr, MII_PHYID1, &phydata);
        }
      while ((ret < 0 || phydata == 0xffff) && ++retries < 3);

      /* If we successfully read anything then break out, using this PHY address */

      if (retries < 3)
        {
          break;
        }
    }

  if (phyaddr >= 32)
    {
      nerr("ERROR: Failed to read %s PHYID1 at any address\n");
      return -ENOENT;
    }

  ninfo("%s: Using PHY address %u\n", BOARD_PHY_NAME, phyaddr);
  priv->phyaddr = phyaddr;

  /* Verify PHYID1.  Compare OUI bits 3-18 */

  ninfo("%s: PHYID1: %04x\n", BOARD_PHY_NAME, phydata);
  if (phydata != BOARD_PHYID1)
    {
      nerr("ERROR: PHYID1=%04x incorrect for %s.  Expected %04x\n",
           phydata, BOARD_PHY_NAME, BOARD_PHYID1);
      return -ENXIO;
    }

  /* Read PHYID2 */

  ret = kinetis_readmii(priv, phyaddr, MII_PHYID2, &phydata);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read %s PHYID2: %d\n", BOARD_PHY_NAME, ret);
      return ret;
    }

  ninfo("%s: PHYID2: %04x\n", BOARD_PHY_NAME, phydata);

  /* Verify PHYID2:  Compare OUI bits 19-24 and the 6-bit model number
   * (ignoring the 4-bit revision number).
   */

  if ((phydata & 0xfff0) != (BOARD_PHYID2 & 0xfff0))
    {
      nerr("ERROR: PHYID2=%04x incorrect for %s.  Expected %04x\n",
           (phydata & 0xfff0), BOARD_PHY_NAME, (BOARD_PHYID2 & 0xfff0));
      return -ENXIO;
    }

  /* Start auto negotiation */

  ninfo("%s: Start autonegotiation...\n",  BOARD_PHY_NAME);
  kinetis_writemii(priv, phyaddr, MII_MCR,
                  (MII_MCR_ANRESTART | MII_MCR_ANENABLE));

  /* Wait (potentially forever) for auto negotiation to complete */

  do
    {
      usleep(LINK_WAITUS);
      ret = kinetis_readmii(priv, phyaddr, MII_MSR, &phydata);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read %s MII_MSR: %d\n",
                BOARD_PHY_NAME, ret);
          return ret;
        }
    }
  while ((phydata & MII_MSR_ANEGCOMPLETE) == 0);

  ninfo("%s: Autonegotiation complete\n",  BOARD_PHY_NAME);
  ninfo("%s: MII_MSR: %04x\n", BOARD_PHY_NAME, phydata);

  /* When we get here we have a link - Find the negotiated speed and duplex. */

  phydata = 0;
  ret = kinetis_readmii(priv, phyaddr, BOARD_PHY_STATUS, &phydata);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read %s BOARD_PHY_STATUS{%02x]: %d\n",
           BOARD_PHY_NAME, BOARD_PHY_STATUS, ret);
      return ret;
   }


  ninfo("%s: BOARD_PHY_STATUS: %04x\n", BOARD_PHY_NAME, phydata);

  /* Set up the transmit and receive control registers based on the
   * configuration and the auto negotiation results.
   */

#ifdef CONFIG_KINETIS_ENETUSEMII
  rcr = ENET_RCR_MII_MODE | ENET_RCR_CRCFWD |
        CONFIG_NET_ETH_MTU << ENET_RCR_MAX_FL_SHIFT |
        ENET_RCR_MII_MODE;
#else
  rcr = ENET_RCR_RMII_MODE | ENET_RCR_CRCFWD |
        CONFIG_NET_ETH_MTU << ENET_RCR_MAX_FL_SHIFT |
        ENET_RCR_MII_MODE;
#endif
  tcr = 0;

  putreg32(rcr, KINETIS_ENET_RCR);
  putreg32(tcr, KINETIS_ENET_TCR);

  /* Setup half or full duplex */

  if (BOARD_PHY_ISDUPLEX(phydata))
    {
      /* Full duplex */

      ninfo("%s: Full duplex\n",  BOARD_PHY_NAME);
      tcr |= ENET_TCR_FDEN;
    }
  else
    {
      /* Half duplex */

      ninfo("%s: Half duplex\n",  BOARD_PHY_NAME);
      rcr |= ENET_RCR_DRT;
    }

  if (BOARD_PHY_10BASET(phydata))
    {
      /* 10 Mbps */

      ninfo("%s: 10 Base-T\n",  BOARD_PHY_NAME);
      rcr |= ENET_RCR_RMII_10T;
    }
  else if (!BOARD_PHY_100BASET(phydata))
    {
      /* 100 Mbps */

      ninfo("%s: 100 Base-T\n",  BOARD_PHY_NAME);
    }
  else
    {
      /* This might happen if autonegotiation did not complete(?) */

      nerr("ERROR: Neither 10- nor 100-BaseT reported: PHY STATUS=%04x\n",
           phydata);
      return -EIO;
    }

  putreg32(rcr, KINETIS_ENET_RCR);
  putreg32(tcr, KINETIS_ENET_TCR);
  return OK;
}

/****************************************************************************
 * Function: kinetis_initbuffers
 *
 * Description:
 *   Initialize ENET buffers and descriptors
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void kinetis_initbuffers(struct kinetis_driver_s *priv)
{
  uintptr_t addr;
  int i;

  /* Get an aligned TX descriptor (array)address */

  addr         = ((uintptr_t)priv->desc + 0x0f) & ~0x0f;
  priv->txdesc = (struct enet_desc_s *)addr;

  /* Get an aligned RX descriptor (array) address */

  addr        +=  CONFIG_KINETIS_ENETNTXBUFFERS * sizeof(struct enet_desc_s);
  priv->rxdesc = (struct enet_desc_s *)addr;

  /* Get the beginning of the first aligned buffer */

  addr        = ((uintptr_t)priv->buffers + 0x0f) & ~0x0f;

  /* Then fill in the TX descriptors */

  for (i = 0; i < CONFIG_KINETIS_ENETNTXBUFFERS; i++)
    {
      priv->txdesc[i].status1 = 0;
      priv->txdesc[i].length  = 0;
      priv->txdesc[i].data    = (uint8_t *)kinesis_swap32((uint32_t)addr);
#ifdef CONFIG_KINETIS_ENETENHANCEDBD
      priv->txdesc[i].status2 = TXDESC_IINS | TXDESC_PINS;
#endif
      addr                   += KINETIS_BUF_SIZE;
    }

  /* Then fill in the RX descriptors */

  for (i = 0; i < CONFIG_KINETIS_ENETNRXBUFFERS; i++)
    {
      priv->rxdesc[i].status1 = RXDESC_E;
      priv->rxdesc[i].length  = 0;
      priv->rxdesc[i].data    = (uint8_t *)kinesis_swap32((uint32_t)addr);
#ifdef CONFIG_KINETIS_ENETENHANCEDBD
      priv->rxdesc[i].bdu     = 0;
      priv->rxdesc[i].status2 = RXDESC_INT;
#endif
      addr                   += KINETIS_BUF_SIZE;
    }

  /* Set the wrap bit in the last descriptors to form a ring */

  priv->txdesc[CONFIG_KINETIS_ENETNTXBUFFERS-1].status1 |= TXDESC_W;
  priv->rxdesc[CONFIG_KINETIS_ENETNRXBUFFERS-1].status1 |= RXDESC_W;

  /* We start with RX descriptor 0 and with no TX descriptors in use */

  priv->txtail = 0;
  priv->txhead = 0;
  priv->rxtail = 0;

  /* Initialize the packet buffer, which is used when sending */

  priv->dev.d_buf =
    (uint8_t*)kinesis_swap32((uint32_t)priv->txdesc[priv->txhead].data);
}

/****************************************************************************
 * Function: kinetis_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void kinetis_reset(struct kinetis_driver_s *priv)
{
  unsigned int i;

  /* Set the reset bit and clear the enable bit */

  putreg32(ENET_ECR_RESET, KINETIS_ENET_ECR);

  /* Wait at least 8 clock cycles */

  for (i = 0; i < 10; i++)
    {
      asm volatile ("nop");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: kinetis_netinitialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int kinetis_netinitialize(int intf)
{
  struct kinetis_driver_s *priv;
  uint32_t regval;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_KINETIS_ENETNETHIFS);
  priv = &g_enet[intf];

  /* Enable the ENET clock */

  regval  = getreg32(KINETIS_SIM_SCGC2);
  regval |= SIM_SCGC2_ENET;
  putreg32(regval, KINETIS_SIM_SCGC2);

  /* Allow concurrent access to MPU controller. Example: ENET uDMA to SRAM,
   * otherwise a bus error will result.
   */

  putreg32(0, KINETIS_MPU_CESR);

#ifdef CONFIG_KINETIS_ENETUSEMII
  /* Configure all ENET/MII pins */

  kinetis_pinconfig(PIN_MII0_MDIO);
  kinetis_pinconfig(PIN_MII0_MDC);
  kinetis_pinconfig(PIN_MII0_RXDV);
  kinetis_pinconfig(PIN_MII0_RXER);
  kinetis_pinconfig(PIN_MII0_TXER);
  kinetis_pinconfig(PIN_MII0_RXD0);
  kinetis_pinconfig(PIN_MII0_RXD1);
  kinetis_pinconfig(PIN_MII0_RXD2);
  kinetis_pinconfig(PIN_MII0_RXD3);
  kinetis_pinconfig(PIN_MII0_TXD0);
  kinetis_pinconfig(PIN_MII0_TXD1);
  kinetis_pinconfig(PIN_MII0_TXD3);
  kinetis_pinconfig(PIN_MII0_TXD2);
  kinetis_pinconfig(PIN_MII0_TXEN);
  kinetis_pinconfig(PIN_MII0_RXCLK);
  kinetis_pinconfig(PIN_MII0_TXCLK);
  kinetis_pinconfig(PIN_MII0_CRS);
  kinetis_pinconfig(PIN_MII0_COL);
#else
  /* Use RMII subset */

  kinetis_pinconfig(PIN_RMII0_MDIO);
  kinetis_pinconfig(PIN_RMII0_MDC);
  kinetis_pinconfig(PIN_RMII0_CRS_DV);
  kinetis_pinconfig(PIN_RMII0_RXER);
  kinetis_pinconfig(PIN_RMII0_RXD0);
  kinetis_pinconfig(PIN_RMII0_RXD1);
  kinetis_pinconfig(PIN_RMII0_TXD0);
  kinetis_pinconfig(PIN_RMII0_TXD1);
  kinetis_pinconfig(PIN_RMII0_TXEN);
#endif

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set interrupt priority levels */

  up_prioritize_irq(KINETIS_IRQ_EMACTMR, CONFIG_KINETIS_EMACTMR_PRIO);
  up_prioritize_irq(KINETIS_IRQ_EMACTX, CONFIG_KINETIS_EMACTX_PRIO);
  up_prioritize_irq(KINETIS_IRQ_EMACRX, CONFIG_KINETIS_EMACRX_PRIO);
  up_prioritize_irq(KINETIS_IRQ_EMACMISC, CONFIG_KINETIS_EMACMISC_PRIO);
#endif

  /* Attach the Ethernet MAC IEEE 1588 timer interrupt handler */

#if 0
  if (irq_attach(KINETIS_IRQ_EMACTMR, kinetis_tmrinterrupt))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTMR IRQ\n");
      return -EAGAIN;
    }
#endif

  /* Attach the Ethernet MAC transmit interrupt handler */

  if (irq_attach(KINETIS_IRQ_EMACTX, kinetis_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACTX IRQ\n");
      return -EAGAIN;
    }

  /* Attach the Ethernet MAC receive interrupt handler */

  if (irq_attach(KINETIS_IRQ_EMACRX, kinetis_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACRX IRQ\n");
      return -EAGAIN;
    }

  /* Attach the Ethernet MAC error and misc interrupt handler */

  if (irq_attach(KINETIS_IRQ_EMACMISC, kinetis_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach EMACMISC IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct kinetis_driver_s));
  priv->dev.d_ifup    = kinetis_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = kinetis_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = kinetis_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = kinetis_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = kinetis_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_PHY_IOCTL
  priv->dev.d_ioctl   = kinetis_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = (void *)g_enet;   /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll        = wd_create();      /* Create periodic poll timer */
  priv->txtimeout     = wd_create();      /* Create TX timeout timer */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling kinetis_ifdown().
   */

  (void)kinetis_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if CONFIG_KINETIS_ENETNETHIFS == 1
void up_netinitialize(void)
{
  (void)kinetis_netinitialize(0);
}
#endif

#endif /* KINETIS_NENET > 0 */
#endif /* CONFIG_NET && CONFIG_KINETIS_ENET */
