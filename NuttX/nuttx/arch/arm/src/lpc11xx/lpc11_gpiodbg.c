/****************************************************************************
 * arch/arm/src/lpc11xx/lpc11_gpiodbg.c
 *
 *   Copyright (C) 2010-2011, 2013, 2016 Gregory Nutt. All rights reserved.
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

/* Output debug info even if debug output is not selected. */

#undef  CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_INFO 1

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <nuttx/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "lpc11_gpio.h"

#ifdef CONFIG_DEBUG_GPIO_INFO

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc11_pinsel
 *
 * Description:
 *   Get the address of the PINSEL register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

#ifdef LPC176x
static uint32_t lpc11_pinsel(unsigned int port, unsigned int pin)
{
  if (pin < 16)
    {
      return g_lopinsel[port];
    }
  else
    {
      return g_hipinsel[port];
    }
}
#endif /* LPC176x */

/****************************************************************************
 * Name: lpc11_pinmode
 *
 * Description:
 *   Get the address of the PINMODE register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

#ifdef LPC176x
static uint32_t lpc11_pinmode(unsigned int port, unsigned int pin)
{
  if (pin < 16)
    {
      return g_lopinmode[port];
    }
  else
    {
      return g_hipinmode[port];
    }
}
#endif /* LPC176x */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  lpc11_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int lpc11_dumpgpio(lpc11_pinset_t pinset, const char *msg)
{
  irqstate_t   flags;
  uint32_t     base;
#if defined(LPC176x)
  uint32_t     pinsel;
  uint32_t     pinmode;
#elif defined(LPC178x)
  uint32_t     iocon;
#endif /* LPC176x */
  unsigned int port;
  unsigned int pin;

  /* Get the base address associated with the GPIO port */

  port    = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin     = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

#if defined(LPC176x)
  pinsel  = lpc11_pinsel(port, pin);
  pinmode = lpc11_pinmode(port, pin);
#elif defined(LPC178x)
  iocon   = LPC11_IOCON_P(port, pin);
#endif /* LPC176x */

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();
  gpioinfo("GPIO%c pin%d (pinset: %08x) -- %s\n",
           port + '0', pin, pinset, msg);

#if defined(LPC176x)
  gpioinfo("  PINSEL[%08x]: %08x PINMODE[%08x]: %08x ODMODE[%08x]: %08x\n",
           pinsel,  pinsel  ? getreg32(pinsel) : 0,
           pinmode, pinmode ? getreg32(pinmode) : 0,
           g_odmode[port],    getreg32(g_odmode[port]));
#elif defined(LPC178x)
  gpioinfo("  IOCON[%08x]: %08x\n", iocon, getreg32(iocon));
#endif

  base = g_fiobase[port];
  gpioinfo("  FIODIR[%08x]: %08x FIOMASK[%08x]: %08x FIOPIN[%08x]: %08x\n",
           base+LPC11_FIO_DIR_OFFSET,  getreg32(base+LPC11_FIO_DIR_OFFSET),
           base+LPC11_FIO_MASK_OFFSET, getreg32(base+LPC11_FIO_MASK_OFFSET),
           base+LPC11_FIO_PIN_OFFSET,  getreg32(base+LPC11_FIO_PIN_OFFSET));

  base = g_intbase[port];
  gpioinfo("  IOINTSTATUS[%08x]: %08x INTSTATR[%08x]: %08x INSTATF[%08x]: %08x\n",
           LPC11_GPIOINT_IOINTSTATUS,          getreg32(LPC11_GPIOINT_IOINTSTATUS),
           base+LPC11_GPIOINT_INTSTATR_OFFSET, getreg32(base+LPC11_GPIOINT_INTSTATR_OFFSET),
           base+LPC11_GPIOINT_INTSTATF_OFFSET, getreg32(base+LPC11_GPIOINT_INTSTATF_OFFSET));
  gpioinfo("  INTENR[%08x]: %08x INTENF[%08x]: %08x\n",
           base+LPC11_GPIOINT_INTENR_OFFSET,   getreg32(base+LPC11_GPIOINT_INTENR_OFFSET),
           base+LPC11_GPIOINT_INTENF_OFFSET,   getreg32(base+LPC11_GPIOINT_INTENF_OFFSET));

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_DEBUG_GPIO_INFO */
