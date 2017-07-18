/****************************************************************************
 * configs/freedom-kl25z/src/kl_tsi.c
 *
 *   Copyright (C) 2014 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <arch/board/kl_wifi.h>

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/include/cc3000_upif.h>

#include "up_arch.h"
#include "kl_gpio.h"
#include "chip/kl_pinmux.h"
#include "chip/kl_sim.h"
#include "freedom-kl25z.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_WL_CC3000
#ifndef CONFIG_DRIVERS_WIRELESS
#  error "Wireless support requires CONFIG_DRIVERS_WIRELESS"
#endif

#ifndef CONFIG_KL_SPI0
#  error "CC3000 Wireless support requires CONFIG_KL_SPI0"
#endif

#ifndef CC3000_SPI_FREQUENCY
#  define CC3000_SPI_FREQUENCY 16000000
#endif

#ifndef CC3000_SPIDEV
#  define CC3000_SPIDEV 0
#endif

#if CC3000_SPIDEV != 0
#  error "CC3000_SPIDEV must be 0"
#endif

#ifndef CC3000_DEVMINOR
#  define CC3000_DEVMINOR 0
#endif

#ifndef CONFIG_CC3000_RX_BUFFER_SIZE
#  define CONFIG_CC3000_RX_BUFFER_SIZE 132
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kl_config_s
{
  struct cc3000_config_s dev;
  xcpt_t handler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the CC3000 driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.  If possible, interrupts should be configured
 * on falling edges to detect the Ready Condition At T2: The normal master
 * SPI write sequence is SPI_CS low, followed by SPI_IRQ low CC3000 to host,
 * indicating that the CC3000 core module is ready to accept data. T2
 * duration is approximately 7 ms.
 *
 *   irq_attach       - Attach the CC3000 interrupt handler to the GPIO
 *                      interrupt
 *   irq_enable       - Enable or disable the GPIO interrupt
 *   irq_clear        - Acknowledge/clear any pending GPIO interrupt
 *   power_enable     - Enable or disable Module enable.
 *   chip_chip_select - The Chip Select
 *   irq_read         - Return the state of the interrupt GPIO input
 *   probe            - Debug support
 */

static int  wl_attach_irq(FAR struct cc3000_config_s *state, xcpt_t handler);
static void wl_enable_irq(FAR struct cc3000_config_s *state, bool enable);
static void wl_clear_irq(FAR struct cc3000_config_s *state);
static void wl_select(FAR struct cc3000_config_s *state, bool enable);
static void wl_enable_power(FAR struct cc3000_config_s *state, bool enable);
static bool wl_read_irq(FAR struct cc3000_config_s *state);
#ifdef CONFIG_CC3000_PROBES
static bool probe(FAR struct cc3000_config_s *state,int n, bool s);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the CC3000
 * driver.  This structure provides information about the configuration
 * of the CC3000 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct kl_config_s g_cc3000_info =
{
  .dev.spi_frequency    = CONFIG_CC3000_SPI_FREQUENCY,
  .dev.spi_mode         = CONFIG_CC3000_SPI_MODE,
  .dev.max_rx_size      = 0,
  .dev.irq_attach       = wl_attach_irq,
  .dev.irq_enable       = wl_enable_irq,
  .dev.irq_clear        = wl_clear_irq,
  .dev.power_enable     = wl_enable_power,
  .dev.chip_chip_select = wl_select,
  .dev.irq_read         = wl_read_irq,
#ifdef CONFIG_CC3000_PROBES
  .dev.probe            = probe, /* This is used for debugging */
#endif
  .handler              = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the CC3000 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   irq_attach       - Attach the CC3000 interrupt handler to the GPIO
 *                      interrupt
 *   irq_enable       - Enable or disable the GPIO interrupt
 *   irq_clear        - Acknowledge/clear any pending GPIO interrupt
 *   power_enable     - Enable or disable Module enable.
 *   chip_chip_select - The Chip Select
 *   irq_read         - Return the state of the interrupt GPIO input
 *   probe            - Debug support
 */

static int wl_attach_irq(FAR struct cc3000_config_s *state, xcpt_t handler)
{
  FAR struct kl_config_s *priv = (FAR struct kl_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void wl_enable_irq(FAR struct cc3000_config_s *state, bool enable)
{
  FAR struct kl_config_s *priv = (FAR struct kl_config_s *)state;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv->handler || !enable);

  /* Attach and enable, or detach and disable */

  iinfo("enable:%d\n", enable);
  if (enable)
    {
      (void)kl_gpioirqattach(GPIO_WIFI_INT, priv->handler);
      kl_gpioirqenable(GPIO_WIFI_INT); 
    }
  else
    {
      (void)kl_gpioirqattach(GPIO_WIFI_INT, NULL);
      kl_gpioirqdisable(GPIO_WIFI_INT); 
    }
}

static void wl_enable_power(FAR struct cc3000_config_s *state, bool enable)
{
  iinfo("enable:%d\n", enable);

  /* Active high enable */

  kl_gpiowrite(GPIO_WIFI_EN, enable);
}

static void wl_select(FAR struct cc3000_config_s *state, bool enable)
{
  iinfo("enable:%d\n", enable);

  /* Active high enable */

  kl_gpiowrite(GPIO_WIFI_CS, enable);
}

static void wl_clear_irq(FAR struct cc3000_config_s *state)
{
  /* Does nothing */
}

static bool wl_read_irq(FAR struct cc3000_config_s *state)
{
  /* Active low*/

  return  kl_gpioread(GPIO_WIFI_INT) ? false : true;
}

#ifdef CONFIG_CC3000_PROBES
static bool probe(FAR struct cc3000_config_s *state,int n, bool s)
{
  if (n == 0)
    {
      kl_gpiowrite(GPIO_D0, s);
    }

  if (n == 1)
    {
      kl_gpiowrite(GPIO_D1, s);
    }

  return true;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_wlinitialize
 *
 * Description:
 *   Each board that supports a wireless device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the wireless device.  This function will register the driver
 *   as /dev/wirelessN where N is the minor device number.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int wireless_archinitialize(size_t max_rx_size)
{
  FAR struct spi_dev_s *spi;

  /* Init SPI bus */

  iinfo("minor %d\n", minor);
  DEBUGASSERT(CONFIG_CC3000_DEVMINOR == 0);

#ifdef CONFIG_CC3000_PROBES
  kl_configgpio(GPIO_D0);
  kl_configgpio(GPIO_D1);
  kl_gpiowrite(GPIO_D0, 1);
  kl_gpiowrite(GPIO_D1, 1);
#endif

  /* Get an instance of the SPI interface */

  spi = kl_spibus_initialize(CONFIG_CC3000_SPIDEV);
  if (!spi)
    {
      ierr("ERROR: Failed to initialize SPI bus %d\n", CONFIG_CC3000_SPIDEV);
      return -ENODEV;
    }

  /* Initialize and register the SPI CC3000 device */

  g_cc3000_info.dev.max_rx_size = max_rx_size ? max_rx_size : CONFIG_CC3000_RX_BUFFER_SIZE;
  int ret = cc3000_register(spi, &g_cc3000_info.dev, CONFIG_CC3000_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to initialize SPI bus %d\n", CONFIG_CC3000_SPIDEV);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: C3000_wlan_init
 *
 * Description:
 *   Initialize wlan driver
 *
 *   Warning: This function must be called before ANY other wlan driver
 *   function
 *
 * Input Parmeters:
 *   sWlanCB   Asynchronous events callback.
 *             0 no event call back.
 *             - Call back parameters:
 *               1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect event,
 *                  HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event,
 *                  HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done,
 *                  HCI_EVNT_WLAN_UNSOL_DHCP dhcp report,
 *                  HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR
 *                  HCI_EVNT_WLAN_KEEPALIVE keepalive.
 *               2) data: pointer to extra data that received by the event
 *                  (NULL no data).
 *               3) length: data length.
 *              - Events with extra data:
 *                  HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask,
 *                    4 bytes default gateway, 4 bytes DHCP server and 4 bytes
 *                    for DNS server.
 *                  HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent,
 *                    4 bytes Packets received, 4 bytes Min round time,
 *                    4 bytes Max round time and 4 bytes for Avg round time.
 *
 *     sFWPatches  0 no patch or pointer to FW patches
 *     sDriverPatches  0 no patch or pointer to driver patches
 *     sBootLoaderPatches  0 no patch or pointer to bootloader patches
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cc3000_wlan_init(size_t max_tx_len,
                      tWlanCB sWlanCB,
                      tFWPatches sFWPatches, tDriverPatches
                      sDriverPatches,  tBootLoaderPatches sBootLoaderPatches)
{
  wlan_init(max_tx_len, sWlanCB, sFWPatches, sDriverPatches, sBootLoaderPatches);
}

#endif /* CONFIG_WL_CC3000 */
