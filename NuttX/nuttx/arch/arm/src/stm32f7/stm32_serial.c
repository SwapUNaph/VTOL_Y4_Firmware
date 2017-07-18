/****************************************************************************
 * arch/arm/src/stm32f7/stm32_serial.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/serial.h>

#include "cache.h"
#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "stm32_gpio.h"
#include "chip/stm32_pinmap.h"
#include "stm32_dma.h"
#include "stm32_rcc.h"
#include "stm32_uart.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Some sanity checks *******************************************************/
/* Total number of possible serial devices */

#define STM32_NSERIAL (STM32F7_NUSART + STM32F7_NUART)

/* DMA configuration */

/* If DMA is enabled on any USART, then very that other pre-requisites
 * have also been selected.
 */

#ifdef SERIAL_HAVE_DMA

/* Verify that DMA has been enabled and the DMA channel has been defined.
 */

#  if defined(CONFIG_USART1_RXDMA) || defined(CONFIG_USART6_RXDMA)
#    ifndef CONFIG_STM32F7_DMA2
#      error STM32 USART1/6 receive DMA requires CONFIG_STM32F7_DMA2
#    endif
#  endif

#  if defined(CONFIG_USART2_RXDMA) || defined(CONFIG_USART3_RXDMA) || \
      defined(CONFIG_UART4_RXDMA) || defined(CONFIG_UART5_RXDMA) || \
      defined(CONFIG_UART7_RXDMA) || defined(CONFIG_UART8_RXDMA)
#    ifndef CONFIG_STM32F7_DMA1
#      error STM32 USART2/3/4/5/7/8 receive DMA requires CONFIG_STM32F7_DMA1
#    endif
#  endif

/* Currently RS-485 support cannot be enabled when RXDMA is in use due to lack
 * of testing - RS-485 support was developed on STM32F1x
 */

#  if (defined(CONFIG_USART1_RXDMA) && defined(CONFIG_USART1_RS485)) || \
      (defined(CONFIG_USART2_RXDMA) && defined(CONFIG_USART2_RS485)) || \
      (defined(CONFIG_USART3_RXDMA) && defined(CONFIG_USART3_RS485)) || \
      (defined(CONFIG_UART4_RXDMA) && defined(CONFIG_UART4_RS485)) || \
      (defined(CONFIG_UART5_RXDMA) && defined(CONFIG_UART5_RS485)) || \
      (defined(CONFIG_USART6_RXDMA) && defined(CONFIG_USART6_RS485)) || \
      (defined(CONFIG_UART7_RXDMA) && defined(CONFIG_UART7_RS485)) || \
      (defined(CONFIG_UART8_RXDMA) && defined(CONFIG_UART8_RS485))
#    error "RXDMA and RS-485 cannot be enabled at the same time for the same U[S]ART"
#  endif

/* There may be alternate DMA channels for each U[S]ART.  Logic in the
 * board.h file should provide definitions to select among the alternatives.
 */

#  if defined(CONFIG_USART1_RXDMA) && !defined(DMAMAP_USART1_RX)
#    error "USART1 DMA channel not defined (DMAMAP_USART1_RX)"
#  endif

#  if defined(CONFIG_USART2_RXDMA) && !defined(DMAMAP_USART2_RX)
#    error "USART2 DMA channel not defined (DMAMAP_USART2_RX)"
#  endif

#  if defined(CONFIG_USART3_RXDMA) && !defined(DMAMAP_USART3_RX)
#    error "USART3 DMA channel not defined (DMAMAP_USART3_RX)"
#  endif

#  if defined(CONFIG_UART4_RXDMA) && !defined(DMAMAP_UART4_RX)
#    error "UART4 DMA channel not defined (DMAMAP_UART4_RX)"
#  endif

#  if defined(CONFIG_UART5_RXDMA) && !defined(DMAMAP_UART5_RX)
#    error "UART5 DMA channel not defined (DMAMAP_UART5_RX)"
#  endif

#  if defined(CONFIG_USART6_RXDMA) && !defined(DMAMAP_USART6_RX)
#    error "USART6 DMA channel not defined (DMAMAP_USART6_RX)"
#  endif

#  if defined(CONFIG_UART7_RXDMA) && !defined(DMAMAP_UART7_RX)
#    error "UART7 DMA channel not defined (DMAMAP_UART7_RX)"
#  endif

#  if defined(CONFIG_UART8_RXDMA) && !defined(DMAMAP_UART8_RX)
#    error "UART8 DMA channel not defined (DMAMAP_UART8_RX)"
#  endif

/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called
 * every time the FIFO receives half this number of bytes.
 *
 * This buffer size should be an even multiple of the Cortex-M7
 * D-Cache line size so that it can be individually invalidated.
 */

#  define RXDMA_BUFFER_MASK   (ARMV7M_DCACHE_LINESIZE - 1)
#  define RXDMA_BUFFER_SIZE   ((32 + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)

/* DMA priority */

#  ifndef CONFIG_USART_DMAPRIO
#      define CONFIG_USART_DMAPRIO  DMA_SCR_PRIMED
#  endif

#  if (CONFIG_USART_DMAPRIO & ~DMA_SCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_USART_DMAPRIO"
#  endif

/* DMA control words */

#  define SERIAL_DMA_CONTROL_WORD      \
              (DMA_SCR_DIR_P2M       | \
               DMA_SCR_CIRC          | \
               DMA_SCR_MINC          | \
               DMA_SCR_PSIZE_8BITS   | \
               DMA_SCR_MSIZE_8BITS   | \
               CONFIG_USART_DMAPRIO  | \
               DMA_SCR_PBURST_SINGLE | \
               DMA_SCR_MBURST_SINGLE)
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
#    define SERIAL_DMA_IFLOW_CONTROL_WORD \
              (DMA_SCR_DIR_P2M       | \
               DMA_SCR_MINC          | \
               DMA_SCR_PSIZE_8BITS   | \
               DMA_SCR_MSIZE_8BITS   | \
               CONFIG_USART_DMAPRIO  | \
               DMA_SCR_PBURST_SINGLE | \
               DMA_SCR_MBURST_SINGLE)
#  endif
#endif /* SERIAL_HAVE_DMA */

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_PM_SERIAL_ACTIVITY)
#  define CONFIG_PM_SERIAL_ACTIVITY 10
#  define PM_IDLE_DOMAIN             0 /* Revisit */
#endif

/* Keep track if a Break was set
 *
 * Note:
 *
 * 1) This value is set in the priv->ie but never written to the control
 *    register. It must not collide with USART_CR1_USED_INTS or USART_CR3_EIE
 * 2) USART_CR3_EIE is also carried in the up_dev_s ie member.
 *
 * See up_restoreusartint where the masking is done.
 */

#ifdef CONFIG_STM32F7_SERIALBRK_BSDCOMPAT
#  define USART_CR1_IE_BREAK_INPROGRESS_SHFTS 15
#  define USART_CR1_IE_BREAK_INPROGRESS (1 << USART_CR1_IE_BREAK_INPROGRESS_SHFTS)
#endif

#ifdef USE_SERIALDRIVER
#ifdef HAVE_UART

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  struct uart_dev_s dev;       /* Generic UART device */
  uint16_t          ie;        /* Saved interrupt mask bits value */
  uint16_t          sr;        /* Saved status bits */

  /* If termios are supported, then the following fields may vary at
   * runtime.
   */

#ifdef CONFIG_SERIAL_TERMIOS
  uint8_t           parity;    /* 0=none, 1=odd, 2=even */
  uint8_t           bits;      /* Number of bits (7 or 8) */
  bool              stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool              iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool              oflow;     /* output flow control (CTS) enabled */
#endif
  uint32_t          baud;      /* Configured baud */
#else
  const uint8_t     parity;    /* 0=none, 1=odd, 2=even */
  const uint8_t     bits;      /* Number of bits (7 or 8) */
  const bool        stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const bool        iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const bool        oflow;     /* output flow control (CTS) enabled */
#endif
  const uint32_t    baud;      /* Configured baud */
#endif

  const uint8_t     irq;       /* IRQ associated with this USART */
  const uint32_t    apbclock;  /* PCLK 1 or 2 frequency */
  const uint32_t    usartbase; /* Base address of USART registers */
  const uint32_t    tx_gpio;   /* U[S]ART TX GPIO pin configuration */
  const uint32_t    rx_gpio;   /* U[S]ART RX GPIO pin configuration */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const uint32_t    rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t    cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif

#ifdef SERIAL_HAVE_DMA
  const unsigned int rxdma_channel; /* DMA channel assigned */
#endif

  int (*const vector)(int irq, void *context); /* Interrupt handler */

  /* RX DMA state */

#ifdef SERIAL_HAVE_DMA
  DMA_HANDLE        rxdma;     /* currently-open receive DMA stream */
  bool              rxenable;  /* DMA-based reception en/disable */
  uint16_t          rxdmain;   /* Next byte in the DMA where hardware will write */
  uint16_t          rxdmaout;  /* Next byte in the DMA buffer to be read */
  char      *const  rxfifo;    /* Receive DMA buffer */
#endif

#ifdef HAVE_RS485
  const uint32_t    rs485_dir_gpio; /* U[S]ART RS-485 DIR GPIO pin configuration */
  const bool        rs485_dir_polarity; /* U[S]ART RS-485 DIR pin state for TX enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_set_format(struct uart_dev_s *dev);
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt_common(struct up_dev_s *dev);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifndef SERIAL_HAVE_ONLY_DMA
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_DMA
static int  up_dma_setup(struct uart_dev_s *dev);
static void up_dma_shutdown(struct uart_dev_s *dev);
static int  up_dma_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool up_dma_rxavailable(struct uart_dev_s *dev);

static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

#ifdef CONFIG_STM32F7_USART1
static int up_interrupt_usart1(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_USART2
static int up_interrupt_usart2(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_USART3
static int up_interrupt_usart3(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_UART4
static int up_interrupt_uart4(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_UART5
static int up_interrupt_uart5(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_USART6
static int up_interrupt_usart6(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_UART7
static int up_interrupt_uart7(int irq, void *context);
#endif
#ifdef CONFIG_STM32F7_UART8
static int up_interrupt_uart8(int irq, void *context);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,
};
#endif

#ifdef SERIAL_HAVE_DMA
static const struct uart_ops_s g_uart_dma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,
};
#endif

/* DMA buffers.  DMA buffers must:
 *
 * 1. Be a multiple of the D-Cache line size.  This requirement is assured
 *    by the definition of RXDMA buffer size above.
 * 2. Be aligned a D-Cache line boundaries, and
 * 3. Be positioned in DMA-able memory (*NOT* DTCM memory).  This must
 *    be managed by logic in the linker script file.
 *
 * These DMA buffers are defined sequentially here to best assure optimal
 * packing of the buffers.
 */

#ifdef CONFIG_USART1_RXDMA
static char g_usart1rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

# ifdef CONFIG_USART2_RXDMA
static char g_usart2rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_USART3_RXDMA
static char g_usart3rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_UART4_RXDMA
static char g_uart4rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_UART5_RXDMA
static char g_uart5rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_USART6_RXDMA
static char g_usart6rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_UART7_RXDMA
static char g_uart7rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_UART8_RXDMA
static char g_uart8rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

/* Receive/Transmit buffers */

#ifdef CONFIG_STM32F7_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_USART6
static char g_usart6rxbuffer[CONFIG_USART6_RXBUFSIZE];
static char g_usart6txbuffer[CONFIG_USART6_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_UART7
static char g_uart7rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_UART7_TXBUFSIZE];
#endif

#ifdef CONFIG_STM32F7_UART8
static char g_uart8rxbuffer[CONFIG_UART8_RXBUFSIZE];
static char g_uart8txbuffer[CONFIG_UART8_TXBUFSIZE];
#endif

/* This describes the state of the STM32 USART1 ports. */

#ifdef CONFIG_STM32F7_USART1
static struct up_dev_s g_usart1priv =
{
  .dev =
    {
#if CONSOLE_UART == 1
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART1_RXBUFSIZE,
        .buffer  = g_usart1rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART1_TXBUFSIZE,
        .buffer  = g_usart1txbuffer,
      },
#ifdef CONFIG_USART1_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart1priv,
    },

  .irq           = STM32_IRQ_USART1,
  .parity        = CONFIG_USART1_PARITY,
  .bits          = CONFIG_USART1_BITS,
  .stopbits2     = CONFIG_USART1_2STOP,
  .baud          = CONFIG_USART1_BAUD,
  .apbclock      = STM32_PCLK2_FREQUENCY,
  .usartbase     = STM32_USART1_BASE,
  .tx_gpio       = GPIO_USART1_TX,
  .rx_gpio       = GPIO_USART1_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART1_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART1_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART1_RTS,
#endif
#ifdef CONFIG_USART1_RXDMA
  .rxdma_channel = DMAMAP_USART1_RX,
  .rxfifo        = g_usart1rxfifo,
#endif
  .vector        = up_interrupt_usart1,

#ifdef CONFIG_USART1_RS485
  .rs485_dir_gpio = GPIO_USART1_RS485_DIR,
#  if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART2 port. */

#ifdef CONFIG_STM32F7_USART2
static struct up_dev_s g_usart2priv =
{
  .dev =
    {
#if CONSOLE_UART == 2
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART2_RXBUFSIZE,
        .buffer  = g_usart2rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART2_TXBUFSIZE,
        .buffer  = g_usart2txbuffer,
      },
#ifdef CONFIG_USART2_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart2priv,
    },

  .irq           = STM32_IRQ_USART2,
  .parity        = CONFIG_USART2_PARITY,
  .bits          = CONFIG_USART2_BITS,
  .stopbits2     = CONFIG_USART2_2STOP,
  .baud          = CONFIG_USART2_BAUD,
  .apbclock      = STM32_PCLK1_FREQUENCY,
  .usartbase     = STM32_USART2_BASE,
  .tx_gpio       = GPIO_USART2_TX,
  .rx_gpio       = GPIO_USART2_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART2_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART2_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART2_RTS,
#endif
#ifdef CONFIG_USART2_RXDMA
  .rxdma_channel = DMAMAP_USART2_RX,
  .rxfifo        = g_usart2rxfifo,
#endif
  .vector        = up_interrupt_usart2,

#ifdef CONFIG_USART2_RS485
  .rs485_dir_gpio = GPIO_USART2_RS485_DIR,
#  if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART3 port. */

#ifdef CONFIG_STM32F7_USART3
static struct up_dev_s g_usart3priv =
{
  .dev =
    {
#if CONSOLE_UART == 3
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART3_RXBUFSIZE,
        .buffer  = g_usart3rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART3_TXBUFSIZE,
        .buffer  = g_usart3txbuffer,
      },
#ifdef CONFIG_USART3_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart3priv,
    },

  .irq           = STM32_IRQ_USART3,
  .parity        = CONFIG_USART3_PARITY,
  .bits          = CONFIG_USART3_BITS,
  .stopbits2     = CONFIG_USART3_2STOP,
  .baud          = CONFIG_USART3_BAUD,
  .apbclock      = STM32_PCLK1_FREQUENCY,
  .usartbase     = STM32_USART3_BASE,
  .tx_gpio       = GPIO_USART3_TX,
  .rx_gpio       = GPIO_USART3_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART3_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART3_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART3_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART3_RTS,
#endif
#ifdef CONFIG_USART3_RXDMA
  .rxdma_channel = DMAMAP_USART3_RX,
  .rxfifo        = g_usart3rxfifo,
#endif
  .vector        = up_interrupt_usart3,

#ifdef CONFIG_USART3_RS485
  .rs485_dir_gpio = GPIO_USART3_RS485_DIR,
#  if (CONFIG_USART3_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART4 port. */

#ifdef CONFIG_STM32F7_UART4
static struct up_dev_s g_uart4priv =
{
  .dev =
    {
#if CONSOLE_UART == 4
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_UART4_RXBUFSIZE,
        .buffer  = g_uart4rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_UART4_TXBUFSIZE,
        .buffer  = g_uart4txbuffer,
      },
#ifdef CONFIG_UART4_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_uart4priv,
    },

  .irq           = STM32_IRQ_UART4,
  .parity        = CONFIG_UART4_PARITY,
  .bits          = CONFIG_UART4_BITS,
  .stopbits2     = CONFIG_UART4_2STOP,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow         = false,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow         = false,
#endif
  .baud          = CONFIG_UART4_BAUD,
  .apbclock      = STM32_PCLK1_FREQUENCY,
  .usartbase     = STM32_UART4_BASE,
  .tx_gpio       = GPIO_UART4_TX,
  .rx_gpio       = GPIO_UART4_RX,
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .cts_gpio      = 0,
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rts_gpio      = 0,
#endif
#ifdef CONFIG_UART4_RXDMA
  .rxdma_channel = DMAMAP_UART4_RX,
  .rxfifo        = g_uart4rxfifo,
#endif
  .vector        = up_interrupt_uart4,

#ifdef CONFIG_UART4_RS485
  .rs485_dir_gpio = GPIO_UART4_RS485_DIR,
#  if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART5 port. */

#ifdef CONFIG_STM32F7_UART5
static struct up_dev_s g_uart5priv =
{
  .dev =
    {
#if CONSOLE_UART == 5
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_UART5_RXBUFSIZE,
        .buffer = g_uart5rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_UART5_TXBUFSIZE,
        .buffer = g_uart5txbuffer,
      },
#ifdef CONFIG_UART5_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_uart5priv,
    },

  .irq            = STM32_IRQ_UART5,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stopbits2      = CONFIG_UART5_2STOP,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow         = false,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow         = false,
#endif
  .baud           = CONFIG_UART5_BAUD,
  .apbclock       = STM32_PCLK1_FREQUENCY,
  .usartbase      = STM32_UART5_BASE,
  .tx_gpio        = GPIO_UART5_TX,
  .rx_gpio        = GPIO_UART5_RX,
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .cts_gpio      = 0,
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rts_gpio      = 0,
#endif
#ifdef CONFIG_UART5_RXDMA
  .rxdma_channel = DMAMAP_UART5_RX,
  .rxfifo        = g_uart5rxfifo,
#endif
  .vector         = up_interrupt_uart5,

#ifdef CONFIG_UART5_RS485
  .rs485_dir_gpio = GPIO_UART5_RS485_DIR,
#  if (CONFIG_UART5_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART6 port. */

#ifdef CONFIG_STM32F7_USART6
static struct up_dev_s g_usart6priv =
{
  .dev =
    {
#if CONSOLE_UART == 6
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_USART6_RXBUFSIZE,
        .buffer = g_usart6rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_USART6_TXBUFSIZE,
        .buffer = g_usart6txbuffer,
      },
#ifdef CONFIG_USART6_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_usart6priv,
    },

  .irq            = STM32_IRQ_USART6,
  .parity         = CONFIG_USART6_PARITY,
  .bits           = CONFIG_USART6_BITS,
  .stopbits2      = CONFIG_USART6_2STOP,
  .baud           = CONFIG_USART6_BAUD,
  .apbclock       = STM32_PCLK2_FREQUENCY,
  .usartbase      = STM32_USART6_BASE,
  .tx_gpio        = GPIO_USART6_TX,
  .rx_gpio        = GPIO_USART6_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART6_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = GPIO_USART6_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART6_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = GPIO_USART6_RTS,
#endif
#ifdef CONFIG_USART6_RXDMA
  .rxdma_channel = DMAMAP_USART6_RX,
  .rxfifo        = g_usart6rxfifo,
#endif
  .vector         = up_interrupt_usart6,

#ifdef CONFIG_USART6_RS485
  .rs485_dir_gpio = GPIO_USART6_RS485_DIR,
#  if (CONFIG_USART6_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART7 port. */

#ifdef CONFIG_STM32F7_UART7
static struct up_dev_s g_uart7priv =
{
  .dev =
    {
#if CONSOLE_UART == 7
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_UART7_RXBUFSIZE,
        .buffer = g_uart7rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_UART7_TXBUFSIZE,
        .buffer = g_uart7txbuffer,
      },
#ifdef CONFIG_UART7_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_uart7priv,
    },

  .irq            = STM32_IRQ_UART7,
  .parity         = CONFIG_UART7_PARITY,
  .bits           = CONFIG_UART7_BITS,
  .stopbits2      = CONFIG_UART7_2STOP,
  .baud           = CONFIG_UART7_BAUD,
  .apbclock       = STM32_PCLK1_FREQUENCY,
  .usartbase      = STM32_UART7_BASE,
  .tx_gpio        = GPIO_UART7_TX,
  .rx_gpio        = GPIO_UART7_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART7_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = GPIO_UART7_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART7_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = GPIO_UART7_RTS,
#endif
#ifdef CONFIG_UART7_RXDMA
  .rxdma_channel = DMAMAP_UART7_RX,
  .rxfifo        = g_uart7rxfifo,
#endif
  .vector         = up_interrupt_uart7,

#ifdef CONFIG_UART7_RS485
  .rs485_dir_gpio = GPIO_UART7_RS485_DIR,
#  if (CONFIG_UART7_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART8 port. */

#ifdef CONFIG_STM32F7_UART8
static struct up_dev_s g_uart8priv =
{
  .dev =
    {
#if CONSOLE_UART == 8
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_UART8_RXBUFSIZE,
        .buffer = g_uart8rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_UART8_TXBUFSIZE,
        .buffer = g_uart8txbuffer,
      },
#ifdef CONFIG_UART8_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_uart8priv,
    },

  .irq            = STM32_IRQ_UART8,
  .parity         = CONFIG_UART8_PARITY,
  .bits           = CONFIG_UART8_BITS,
  .stopbits2      = CONFIG_UART8_2STOP,
  .baud           = CONFIG_UART8_BAUD,
  .apbclock       = STM32_PCLK1_FREQUENCY,
  .usartbase      = STM32_UART8_BASE,
  .tx_gpio        = GPIO_UART8_TX,
  .rx_gpio        = GPIO_UART8_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART8_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = GPIO_UART8_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART8_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = GPIO_UART8_RTS,
#endif
#ifdef CONFIG_UART8_RXDMA
  .rxdma_channel = DMAMAP_UART8_RX,
  .rxfifo        = g_uart8rxfifo,
#endif
  .vector         = up_interrupt_uart8,

#ifdef CONFIG_UART8_RS485
  .rs485_dir_gpio = GPIO_UART8_RS485_DIR,
#  if (CONFIG_UART8_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This table lets us iterate over the configured USARTs */

static struct up_dev_s * const uart_devs[STM32_NSERIAL] =
{
#ifdef CONFIG_STM32F7_USART1
  [0] = &g_usart1priv,
#endif
#ifdef CONFIG_STM32F7_USART2
  [1] = &g_usart2priv,
#endif
#ifdef CONFIG_STM32F7_USART3
  [2] = &g_usart3priv,
#endif
#ifdef CONFIG_STM32F7_UART4
  [3] = &g_uart4priv,
#endif
#ifdef CONFIG_STM32F7_UART5
  [4] = &g_uart5priv,
#endif
#ifdef CONFIG_STM32F7_USART6
  [5] = &g_usart6priv,
#endif
#ifdef CONFIG_STM32F7_UART7
  [6] = &g_uart7priv,
#endif
#ifdef CONFIG_STM32F7_UART8
  [7] = &g_uart8priv,
#endif
};

#ifdef CONFIG_PM
static  struct pm_callback_s g_serialcb =
{
  .notify  = up_pm_notify,
  .prepare = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static void up_restoreusartint(struct up_dev_s *priv, uint16_t ie)
{
  uint32_t cr;

  /* Save the interrupt mask */

  priv->ie = ie;

  /* And restore the interrupt state (see the interrupt enable/usage table above) */

  cr = up_serialin(priv, STM32_USART_CR1_OFFSET);
  cr &= ~(USART_CR1_USED_INTS);
  cr |= (ie & (USART_CR1_USED_INTS));
  up_serialout(priv, STM32_USART_CR1_OFFSET, cr);

  cr = up_serialin(priv, STM32_USART_CR3_OFFSET);
  cr &= ~USART_CR3_EIE;
  cr |= (ie & USART_CR3_EIE);
  up_serialout(priv, STM32_USART_CR3_OFFSET, cr);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static inline void up_disableusartint(struct up_dev_s *priv, uint16_t *ie)
{
  if (ie)
    {
      uint32_t cr1;
      uint32_t cr3;

      /* USART interrupts:
       *
       * Enable             Status          Meaning                        Usage
       * ------------------ --------------- ------------------------------ ----------
       * USART_CR1_IDLEIE   USART_ISR_IDLE  Idle Line Detected             (not used)
       * USART_CR1_RXNEIE   USART_ISR_RXNE  Received Data Ready to be Read
       * "              "   USART_ISR_ORE   Overrun Error Detected
       * USART_CR1_TCIE     USART_ISR_TC    Transmission Complete          (used only for RS-485)
       * USART_CR1_TXEIE    USART_ISR_TXE   Transmit Data Register Empty
       * USART_CR1_PEIE     USART_ISR_PE    Parity Error
       *
       * USART_CR2_LBDIE    USART_ISR_LBD   Break Flag                     (not used)
       * USART_CR3_EIE      USART_ISR_FE    Framing Error
       * "           "      USART_ISR_NF    Noise Error
       * "           "      USART_ISR_ORE   Overrun Error Detected
       * USART_CR3_CTSIE    USART_ISR_CTS   CTS flag                       (not used)
       */

      cr1 = up_serialin(priv, STM32_USART_CR1_OFFSET);
      cr3 = up_serialin(priv, STM32_USART_CR3_OFFSET);

      /* Return the current interrupt mask value for the used interrupts.  Notice
       * that this depends on the fact that none of the used interrupt enable bits
       * overlap.  This logic would fail if we needed the break interrupt!
       */

      *ie = (cr1 & (USART_CR1_USED_INTS)) | (cr3 & USART_CR3_EIE);
    }

  /* Disable all interrupts */

  up_restoreusartint(priv, 0);
}

/****************************************************************************
 * Name: up_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_nextrx(struct up_dev_s *priv)
{
  size_t dmaresidual;

  dmaresidual = stm32_dmaresidual(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (int)dmaresidual);
}
#endif

/****************************************************************************
 * Name: up_set_format
 *
 * Description:
 *   Set the serial line format and speed.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_set_format(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;
  uint32_t usartdiv8;
  uint32_t cr1;
  uint32_t brr;

  /* In case of oversampling by 8, the equation is:
   *
   *   baud      = 2 * fCK / usartdiv8
   *   usartdiv8 = 2 * fCK / baud
   */

  usartdiv8 = ((priv->apbclock << 1) + (priv->baud >> 1)) / priv->baud;

  /* Baud rate for standard USART (SPI mode included):
   *
   * In case of oversampling by 16, the equation is:
   *   baud       = fCK / usartdiv16
   *   usartdiv16 = fCK / baud
   *              = 2 * usartdiv8
   */

  /* Use oversamply by 8 only if the divisor is small.  But what is small? */

  cr1 = up_serialin(priv, STM32_USART_CR1_OFFSET);
  if (usartdiv8 > 100)
    {
      /* Use usartdiv16 */

      brr  = (usartdiv8 + 1) >> 1;

      /* Clear oversampling by 8 to enable oversampling by 16 */

      cr1 &= ~USART_CR1_OVER8;
    }
  else
    {
      DEBUGASSERT(usartdiv8 >= 8);

      /* Perform mysterious operations on bits 0-3 */

      brr  = ((usartdiv8 & 0xfff0) | ((usartdiv8 & 0x000f) >> 1));

      /* Set oversampling by 8 */

      cr1 |= USART_CR1_OVER8;
    }

  up_serialout(priv, STM32_USART_CR1_OFFSET, cr1);
  up_serialout(priv, STM32_USART_BRR_OFFSET, brr);

  /* Configure parity mode */

  regval  = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M0);

  if (priv->parity == 1)       /* Odd parity */
    {
      regval |= (USART_CR1_PCE | USART_CR1_PS);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      regval |= USART_CR1_PCE;
    }

  /* Configure word length (Default: 8-bits) */

  if (priv->bits == 7)
    {
      regval |= USART_CR1_M1;
    }
  else if (priv->bits == 9)
    {
      regval |= USART_CR1_M0;
    }

  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);

  /* Configure STOP bits */

  regval = up_serialin(priv, STM32_USART_CR2_OFFSET);
  regval &= ~(USART_CR2_STOP_MASK);

  if (priv->stopbits2)
    {
      regval |= USART_CR2_STOP2;
    }

  up_serialout(priv, STM32_USART_CR2_OFFSET, regval);

  /* Configure hardware flow control */

  regval  = up_serialin(priv, STM32_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && !defined(CONFIG_STM32F7_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      regval |= USART_CR3_RTSE;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow && (priv->cts_gpio != 0))
    {
      regval |= USART_CR3_CTSE;
    }
#endif

  up_serialout(priv, STM32_USART_CR3_OFFSET, regval);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Name: up_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the USART peripheral
 *
 * Input parameters:
 *   dev - A reference to the UART driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void up_set_apb_clock(struct uart_dev_s *dev, bool on)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rcc_en;
  uint32_t regaddr;

  /* Determine which USART to configure */

  switch (priv->usartbase)
    {
    default:
      return;
#ifdef CONFIG_STM32F7_USART1
    case STM32_USART1_BASE:
      rcc_en = RCC_APB2ENR_USART1EN;
      regaddr = STM32_RCC_APB2ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_USART2
    case STM32_USART2_BASE:
      rcc_en = RCC_APB1ENR_USART2EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_USART3
    case STM32_USART3_BASE:
      rcc_en = RCC_APB1ENR_USART3EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_UART4
    case STM32_UART4_BASE:
      rcc_en = RCC_APB1ENR_UART4EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_UART5
    case STM32_UART5_BASE:
      rcc_en = RCC_APB1ENR_UART5EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_USART6
    case STM32_USART6_BASE:
      rcc_en = RCC_APB2ENR_USART6EN;
      regaddr = STM32_RCC_APB2ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_UART7
    case STM32_UART7_BASE:
      rcc_en = RCC_APB1ENR_UART7EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32F7_UART8
    case STM32_UART8_BASE:
      rcc_en = RCC_APB1ENR_UART8EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
    }

  /* Enable/disable APB 1/2 clock for USART */

  if (on)
    {
      modifyreg32(regaddr, 0, rcc_en);
    }
  else
    {
      modifyreg32(regaddr, rcc_en, 0);
    }
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled in stm32_lowsetup().
   */

  /* Enable USART APB1/2 clock */

  up_set_apb_clock(dev, true);

  /* Configure pins for USART use */

  stm32_configgpio(priv->tx_gpio);
  stm32_configgpio(priv->rx_gpio);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      stm32_configgpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      uint32_t config = priv->rts_gpio;

#ifdef CONFIG_STM32F7_FLOWCONTROL_BROKEN
      /* Instead of letting hw manage this pin, we will bitbang */

      config = (config & ~GPIO_MODE_MASK) | GPIO_OUTPUT;
#endif
      stm32_configgpio(config);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32_configgpio(priv->rs485_dir_gpio);
      stm32_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif

  /* Configure CR2 */
  /* Clear STOP, CLKEN, CPOL, CPHA, LBCL, and interrupt enable bits */

  regval  = up_serialin(priv, STM32_USART_CR2_OFFSET);
  regval &= ~(USART_CR2_STOP_MASK | USART_CR2_CLKEN | USART_CR2_CPOL |
              USART_CR2_CPHA | USART_CR2_LBCL | USART_CR2_LBDIE);

  /* Configure STOP bits */

  if (priv->stopbits2)
    {
      regval |= USART_CR2_STOP2;
    }

  up_serialout(priv, STM32_USART_CR2_OFFSET, regval);

  /* Configure CR1 */
  /* Clear TE, REm and all interrupt enable bits */

  regval  = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_TE | USART_CR1_RE | USART_CR1_ALLINTS);

  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);

  /* Configure CR3 */
  /* Clear CTSE, RTSE, and all interrupt enable bits */

  regval  = up_serialin(priv, STM32_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_CTSIE | USART_CR3_CTSE | USART_CR3_RTSE | USART_CR3_EIE);

  up_serialout(priv, STM32_USART_CR3_OFFSET, regval);

  /* Configure the USART line format and speed. */

  up_set_format(dev);

  /* Enable Rx, Tx, and the USART */

  regval      = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval     |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);

#endif /* CONFIG_SUPPRESS_UART_CONFIG */

  /* Set up the cached interrupt enables value */

  priv->ie    = 0;
  return OK;
}

/****************************************************************************
 * Name: up_dma_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int result;
  uint32_t regval;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = up_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

  /* Acquire the DMA channel.  This should always succeed. */

  priv->rxdma = stm32_dmachannel(priv->rxdma_channel);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Configure for non-circular DMA reception into the RX FIFO */

      stm32_dmasetup(priv->rxdma,
                     priv->usartbase + STM32_USART_RDR_OFFSET,
                     (uint32_t)priv->rxfifo,
                     RXDMA_BUFFER_SIZE,
                     SERIAL_DMA_IFLOW_CONTROL_WORD);
    }
  else
#endif
    {
      /* Configure for circular DMA reception into the RX FIFO */

      stm32_dmasetup(priv->rxdma,
                     priv->usartbase + STM32_USART_RDR_OFFSET,
                     (uint32_t)priv->rxfifo,
                     RXDMA_BUFFER_SIZE,
                     SERIAL_DMA_CONTROL_WORD);
    }

  /* Reset our DMA shadow pointer to match the address just
   * programmed above.
   */

  priv->rxdmaout = 0;
  priv->rxdmain  = 0;

  /* Enable receive DMA for the UART */

  regval  = up_serialin(priv, STM32_USART_CR3_OFFSET);
  regval |= USART_CR3_DMAR;
  up_serialout(priv, STM32_USART_CR3_OFFSET, regval);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Start the DMA channel, and arrange for callbacks at the full point
       * in the FIFO. After buffer gets full, hardware flow-control kicks
       * in and DMA transfer is stopped.
       */

      stm32_dmastart(priv->rxdma, up_dma_rxcallback, (void *)priv, false);
    }
  else
#endif
    {
      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      stm32_dmastart(priv->rxdma, up_dma_rxcallback, (void *)priv, true);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;

  /* Disable all interrupts */

  up_disableusartint(priv, NULL);

  /* Disable USART APB1/2 clock */

  up_set_apb_clock(dev, false);

  /* Disable Rx, Tx, and the UART */

  regval  = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);

  /* Release pins. "If the serial-attached device is powered down, the TX
   * pin causes back-powering, potentially confusing the device to the point
   * of complete lock-up."
   *
   * REVISIT:  Is unconfiguring the pins appropriate for all device?  If not,
   * then this may need to be a configuration option.
   */

  stm32_unconfiggpio(priv->tx_gpio);
  stm32_unconfiggpio(priv->rx_gpio);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      stm32_unconfiggpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      stm32_unconfiggpio(priv->rts_gpio);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32_unconfiggpio(priv->rs485_dir_gpio);
    }
#endif
}

/****************************************************************************
 * Name: up_dma_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Perform the normal UART shutdown */

  up_shutdown(dev);

  /* Stop the DMA channel */

  stm32_dmastop(priv->rxdma);

  /* Release the DMA channel */

  stm32_dmafree(priv->rxdma);
  priv->rxdma = NULL;
}
#endif

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, priv->vector);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

       up_enable_irq(priv->irq);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt_common
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt_common(struct up_dev_s *priv)
{
  int  passes;
  bool handled;

  /* Report serial activity to the power management logic */

#if defined(CONFIG_PM) && CONFIG_PM_SERIAL_ACTIVITY > 0
  pm_activity(PM_IDLE_DOMAIN, CONFIG_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked USART status word. */

      priv->sr = up_serialin(priv, STM32_USART_ISR_OFFSET);

      /* USART interrupts:
       *
       * Enable             Status          Meaning                         Usage
       * ------------------ --------------- ------------------------------- ----------
       * USART_CR1_IDLEIE   USART_ISR_IDLE  Idle Line Detected              (not used)
       * USART_CR1_RXNEIE   USART_ISR_RXNE  Received Data Ready to be Read
       * "              "   USART_ISR_ORE   Overrun Error Detected
       * USART_CR1_TCIE     USART_ISR_TC    Transmission Complete           (used only for RS-485)
       * USART_CR1_TXEIE    USART_ISR_TXE   Transmit Data Register Empty
       * USART_CR1_PEIE     USART_ISR_PE    Parity Error
       *
       * USART_CR2_LBDIE    USART_ISR_LBD   Break Flag                      (not used)
       * USART_CR3_EIE      USART_ISR_FE    Framing Error
       * "           "      USART_ISR_NF    Noise Error
       * "           "      USART_ISR_ORE   Overrun Error Detected
       * USART_CR3_CTSIE    USART_ISR_CTS   CTS flag                        (not used)
       *
       * NOTE: Some of these status bits must be cleared by explicitly writing zero
       * to the SR register: USART_ISR_CTS, USART_ISR_LBD. Note of those are currently
       * being used.
       */

#ifdef HAVE_RS485
      /* Transmission of whole buffer is over - TC is set, TXEIE is cleared.
       * Note - this should be first, to have the most recent TC bit value from
       * SR register - sending data affects TC, but without refresh we will not
       * know that...
       */

      if ((priv->sr & USART_ISR_TC) != 0 && (priv->ie & USART_CR1_TCIE) != 0 &&
          (priv->ie & USART_CR1_TXEIE) == 0)
        {
          stm32_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
          up_restoreusartint(priv, priv->ie & ~USART_CR1_TCIE);
        }
#endif

      /* Handle incoming, receive bytes. */

      if ((priv->sr & USART_ISR_RXNE) != 0 && (priv->ie & USART_CR1_RXNEIE) != 0)
        {
          /* Received data ready... process incoming bytes.  NOTE the check for
           * RXNEIE:  We cannot call uart_recvchards of RX interrupts are disabled.
           */

          uart_recvchars(&priv->dev);
          handled = true;
        }

      /* We may still have to read from the DR register to clear any pending
       * error conditions.
       */

      else if ((priv->sr & (USART_ISR_ORE | USART_ISR_NF | USART_ISR_FE)) != 0)
        {
          /* These errors are cleared by writing the corresponding bit to the
           * interrupt clear register (ICR).
           */

          up_serialout(priv, STM32_USART_ICR_OFFSET,
                      (USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_FECF));
        }

      /* Handle outgoing, transmit bytes */

      if ((priv->sr & USART_ISR_TXE) != 0 && (priv->ie & USART_CR1_TXEIE) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(&priv->dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT) \
    || defined(CONFIG_STM32F7_SERIALBRK_BSDCOMPAT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_STM32F7_SERIALBRK_BSDCOMPAT)
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct up_dev_s *user = (struct up_dev_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct up_dev_s));
          }
      }
      break;
#endif

#ifdef CONFIG_STM32F7_USART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        /* Change the TX port to be open-drain/push-pull and enable/disable
         * half-duplex mode.
         */

        uint32_t cr = up_serialin(priv, STM32_USART_CR3_OFFSET);

        if (arg == SER_SINGLEWIRE_ENABLED)
          {
            stm32_configgpio(priv->tx_gpio | GPIO_OPENDRAIN);
            cr |= USART_CR3_HDSEL;
          }
        else
          {
            stm32_configgpio(priv->tx_gpio | GPIO_PUSHPULL);
            cr &= ~USART_CR3_HDSEL;
          }

        up_serialout(priv, STM32_USART_CR3_OFFSET, cr);
      }
     break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        cfsetispeed(termiosp, priv->baud);

        /* Note that since we only support 8/9 bit modes and
         * there is no way to report 9-bit mode, we always claim 8.
         */

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
          CS8;

        /* TODO: CCTS_IFLOW, CCTS_OFLOW */
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

        if (((termiosp->c_cflag & CSIZE) != CS8)
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
           )
          {
            ret = -EINVAL;
            break;
          }

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

        /* Note that since there is no way to request 9-bit mode
         * and no way to support 5/6/7-bit modes, we ignore them
         * all here.
         */

        /* Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

        /* Effect the changes immediately - note that we do not implement
         * TCSADRAIN / TCSAFLUSH
         */

        up_set_format(dev);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_STM32F7_USART_BREAKS
#  ifdef CONFIG_STM32F7_SERIALBRK_BSDCOMPAT
    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags;
        uint32_t tx_break;

        flags = enter_critical_section();

        /* Disable any further tx activity */

        priv->ie |= USART_CR1_IE_BREAK_INPROGRESS;

        up_txint(dev, false);

        /* Configure TX as a GPIO output pin and Send a break signal*/

        tx_break = GPIO_OUTPUT | (~(GPIO_MODE_MASK|GPIO_OUTPUT_SET) & priv->tx_gpio);
        stm32_configgpio(tx_break);

        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;

        flags = enter_critical_section();

        /* Configure TX back to U(S)ART */

        stm32_configgpio(priv->tx_gpio);

        priv->ie &= ~USART_CR1_IE_BREAK_INPROGRESS;

        /* Enable further tx activity */

        up_txint(dev, true);

        leave_critical_section(flags);
      }
      break;
#  else
    case TIOCSBRK:  /* No BSD compatibility: Turn break on for M bit times */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        cr1   = up_serialin(priv, STM32_USART_CR1_OFFSET);
        up_serialout(priv, STM32_USART_CR1_OFFSET, cr1 | USART_CR1_SBK);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* No BSD compatibility: May turn off break too soon */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        cr1   = up_serialin(priv, STM32_USART_CR1_OFFSET);
        up_serialout(priv, STM32_USART_CR1_OFFSET, cr1 & ~USART_CR1_SBK);
        leave_critical_section(flags);
      }
      break;
#  endif
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rdr;

  /* Get the Rx byte */

  rdr      = up_serialin(priv, STM32_USART_RDR_OFFSET);

  /* Get the Rx byte plux error information.  Return those in status */

  *status  = priv->sr << 16 | rdr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return rdr & 0xff;
}
#endif

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint16_t ie;

  /* USART receive interrupts:
   *
   * Enable             Status          Meaning                         Usage
   * ------------------ --------------- ------------------------------- ----------
   * USART_CR1_IDLEIE   USART_ISR_IDLE  Idle Line Detected              (not used)
   * USART_CR1_RXNEIE   USART_ISR_RXNE  Received Data Ready to be Read
   * "              "   USART_ISR_ORE   Overrun Error Detected
   * USART_CR1_PEIE     USART_ISR_PE    Parity Error
   *
   * USART_CR2_LBDIE    USART_ISR_LBD   Break Flag                      (not used)
   * USART_CR3_EIE      USART_ISR_FE    Framing Error
   * "           "      USART_ISR_NF    Noise Error
   * "           "      USART_ISR_ORE   Overrun Error Detected
   */

  flags = enter_critical_section();
  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_USART_ERRINTS
      ie |= (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE);
#else
      ie |= USART_CR1_RXNEIE;
#endif
#endif
    }
  else
    {
      ie &= ~(USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE);
    }

  /* Then set the new interrupt state */

  up_restoreusartint(priv, ie);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, STM32_USART_ISR_OFFSET) & USART_ISR_RXNE) != 0);
}
#endif

/****************************************************************************
 * Name: up_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS) && \
    defined(CONFIG_STM32F7_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      stm32_gpiowrite(priv->rts_gpio, upper);
      return upper;
    }

#else
  if (priv->iflow)
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          uart_disablerxint(dev);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */

          uart_enablerxint(dev);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: up_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rxdmain;
  int c = 0;

  /* If additional bytes have been added to the DMA buffer, then we will need
   * to invalidate the DMA buffer before reading the byte.
   */

  rxdmain = up_dma_nextrx(priv);
  if (rxdmain != priv->rxdmain)
    {
      /* Invalidate the DMA buffer */

      arch_invalidate_dcache((uintptr_t)priv->rxfifo,
                             (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE - 1);

      /* Since DMA is ongoing, there are lots of race conditions here.  We
       * just have to hope that the rxdmaout stays well ahead of rxdmain.
       */

      priv->rxdmain = rxdmain;
    }

  /* Now check if there are any bytes to read from the DMA buffer */

  if (rxdmain != priv->rxdmaout)
    {
      c = priv->rxfifo[priv->rxdmaout];

      priv->rxdmaout++;
      if (priv->rxdmaout == RXDMA_BUFFER_SIZE)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow)
            {
              /* RX DMA buffer full. RX paused, RTS line pulled up to prevent
               * more input data from other end.
               */
            }
          else
#endif
            {
              priv->rxdmaout = 0;
            }
        }
    }

  return c;
}
#endif

/****************************************************************************
 * Name: up_dma_reenable
 *
 * Description:
 *   Call to re-enable RX DMA.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_DMA) && defined(CONFIG_SERIAL_IFLOWCONTROL)
static void up_dma_reenable(struct up_dev_s *priv)
{
  /* Configure for non-circular DMA reception into the RX FIFO */

  stm32_dmasetup(priv->rxdma,
                 priv->usartbase + STM32_USART_RDR_OFFSET,
                 (uint32_t)priv->rxfifo,
                 RXDMA_BUFFER_SIZE,
                 SERIAL_DMA_IFLOW_CONTROL_WORD);

  /* Reset our DMA shadow pointer to match the address just programmed
   * above.
   */

  priv->rxdmaout = 0;
  priv->rxdmain  = 0;

  /* Start the DMA channel, and arrange for callbacks at the full point in
   * the FIFO. After buffer gets full, hardware flow-control kicks in and
   * DMA transfer is stopped.
   */

  stm32_dmastart(priv->rxdma, up_dma_rxcallback, (void *)priv, false);
}
#endif

/****************************************************************************
 * Name: up_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* En/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow && priv->rxenable && (priv->rxdmaout == RXDMA_BUFFER_SIZE))
    {
      /* Re-enable RX DMA. */

      up_dma_reenable(priv);
    }
#endif
}
#endif

/****************************************************************************
 * Name: up_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static bool up_dma_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (up_dma_nextrx(priv) != priv->rxdmaout);
}
#endif

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32_gpiowrite(priv->rs485_dir_gpio, priv->rs485_dir_polarity);
    }
#endif

  up_serialout(priv, STM32_USART_TDR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* USART transmit interrupts:
   *
   * Enable             Status          Meaning                      Usage
   * ------------------ --------------- ---------------------------- ----------
   * USART_CR1_TCIE     USART_ISR_TC    Transmission Complete        (used only for RS-485)
   * USART_CR1_TXEIE    USART_ISR_TXE   Transmit Data Register Empty
   * USART_CR3_CTSIE    USART_ISR_CTS   CTS flag                     (not used)
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint16_t ie = priv->ie | USART_CR1_TXEIE;

      /* If RS-485 is supported on this U[S]ART, then also enable the
       * transmission complete interrupt.
       */

#  ifdef HAVE_RS485
      if (priv->rs485_dir_gpio != 0)
        {
          ie |= USART_CR1_TCIE;
        }
#  endif

#  ifdef CONFIG_STM32_SERIALBRK_BSDCOMPAT
      if (priv->ie & USART_CR1_IE_BREAK_INPROGRESS)
        {
          return;
        }
#  endif

      up_restoreusartint(priv, ie);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      up_restoreusartint(priv, priv->ie & ~USART_CR1_TXEIE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, STM32_USART_ISR_OFFSET) & USART_ISR_TXE) != 0);
}

/****************************************************************************
 * Name: up_interrupt_u[s]art[n]
 *
 * Description:
 *   Interrupt handlers for U[S]ART[n] where n=1,..,6.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_USART1
static int up_interrupt_usart1(int irq, void *context)
{
  return up_interrupt_common(&g_usart1priv);
}
#endif

#ifdef CONFIG_STM32F7_USART2
static int up_interrupt_usart2(int irq, void *context)
{
  return up_interrupt_common(&g_usart2priv);
}
#endif

#ifdef CONFIG_STM32F7_USART3
static int up_interrupt_usart3(int irq, void *context)
{
  return up_interrupt_common(&g_usart3priv);
}
#endif

#ifdef CONFIG_STM32F7_UART4
static int up_interrupt_uart4(int irq, void *context)
{
  return up_interrupt_common(&g_uart4priv);
}
#endif

#ifdef CONFIG_STM32F7_UART5
static int up_interrupt_uart5(int irq, void *context)
{
  return up_interrupt_common(&g_uart5priv);
}
#endif

#ifdef CONFIG_STM32F7_USART6
static int up_interrupt_usart6(int irq, void *context)
{
  return up_interrupt_common(&g_usart6priv);
}
#endif

#ifdef CONFIG_STM32F7_UART7
static int up_interrupt_uart7(int irq, void *context)
{
  return up_interrupt_common(&g_uart7priv);
}
#endif

#ifdef CONFIG_STM32F7_UART8
static int up_interrupt_uart8(int irq, void *context)
{
  return up_interrupt_common(&g_uart8priv);
}
#endif

/****************************************************************************
 * Name: up_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct up_dev_s *priv = (struct up_dev_s *)arg;

  if (priv->rxenable && up_dma_rxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow && priv->rxenable &&
          (priv->rxdmaout == RXDMA_BUFFER_SIZE))
        {
          /* Re-enable RX DMA. */

          up_dma_reenable(priv);
        }
#endif
    }
}
#endif

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */

        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */

        }
        break;

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */

        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */

        }
        break;

      default:
        /* Should not get here */
        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif
#endif /* HAVE_UART */
#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
#ifdef HAVE_UART
  unsigned i;

  /* Disable all USART interrupts */

  for (i = 0; i < STM32_NSERIAL; i++)
    {
      if (uart_devs[i])
        {
          up_disableusartint(uart_devs[i], NULL);
        }
    }

  /* Configure whichever one is the console */

#if CONSOLE_UART > 0
  up_setup(&uart_devs[CONSOLE_UART - 1]->dev);
#endif
#endif /* HAVE UART */
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef HAVE_UART
  char devname[16];
  unsigned i;
  unsigned minor = 0;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_serialcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

  /* Register the console */

#if CONSOLE_UART > 0
  (void)uart_register("/dev/console", &uart_devs[CONSOLE_UART - 1]->dev);

#ifndef CONFIG_SERIAL_DISABLE_REORDERING
  /* If not disabled, register the console UART to ttyS0 and exclude
   * it from initializing it further down
   */

  (void)uart_register("/dev/ttyS0", &uart_devs[CONSOLE_UART - 1]->dev);
  minor = 1;
#endif

#ifdef SERIAL_HAVE_CONSOLE_DMA
  /* If we need to re-initialise the console to enable DMA do that here. */

  up_dma_setup(&uart_devs[CONSOLE_UART - 1]->dev);
#endif
#endif /* CONSOLE_UART > 0 */

  /* Register all remaining USARTs */

  strcpy(devname, "/dev/ttySx");

  for (i = 0; i < STM32_NSERIAL; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (uart_devs[i] == 0)
        {
          continue;
        }

#ifndef CONFIG_SERIAL_DISABLE_REORDERING
      /* Don't create a device for the console - we did that above */

      if (uart_devs[i]->dev.isconsole)
        {
          continue;
        }
#endif

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + minor++;
      (void)uart_register(devname, &uart_devs[i]->dev);
    }
#endif /* HAVE UART */
}

/****************************************************************************
 * Name: stm32_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
void stm32_serial_dma_poll(void)
{
    irqstate_t flags;

    flags = enter_critical_section();

#ifdef CONFIG_USART1_RXDMA
  if (g_usart1priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart1priv.rxdma, 0, &g_usart1priv);
    }
#endif

#ifdef CONFIG_USART2_RXDMA
  if (g_usart2priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart2priv.rxdma, 0, &g_usart2priv);
    }
#endif

#ifdef CONFIG_USART3_RXDMA
  if (g_usart3priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart3priv.rxdma, 0, &g_usart3priv);
    }
#endif

#ifdef CONFIG_UART4_RXDMA
  if (g_uart4priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart4priv.rxdma, 0, &g_uart4priv);
    }
#endif

#ifdef CONFIG_UART5_RXDMA
  if (g_uart5priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart5priv.rxdma, 0, &g_uart5priv);
    }
#endif

#ifdef CONFIG_USART6_RXDMA
  if (g_usart6priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart6priv.rxdma, 0, &g_usart6priv);
    }
#endif

#ifdef CONFIG_UART7_RXDMA
  if (g_uart7priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart7priv.rxdma, 0, &g_uart7priv);
    }
#endif

#ifdef CONFIG_UART8_RXDMA
  if (g_uart8priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart8priv.rxdma, 0, &g_uart8priv);
    }
#endif

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct up_dev_s *priv = uart_devs[CONSOLE_UART - 1];
  uint16_t ie;

  up_disableusartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(priv, ie);
#endif
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_UART > 0
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
