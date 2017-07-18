/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz-gpio.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/pic32mz/irq.h>

#include "up_arch.h"
#include "up_internal.h"
#include "chip/pic32mz-ioport.h"
#include "pic32mz-gpio.h"

#ifdef CONFIG_PIC32MZ_GPIOIRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline unsigned int pic32mz_ioport(pinset_t pinset);
static inline unsigned int pic32mz_pin(pinset_t pinset);
static inline bool pic32mz_input(pinset_t pinset);
static inline bool pic32mz_interrupt(pinset_t pinset);
static inline bool pic32mz_pullup(pinset_t pinset);
static inline bool pic32mz_pulldown(pinset_t pinset);
static int pic32mz_cninterrupt(int irq, FAR void *context);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ioport_level2_s
{
  xcpt_t handler[16];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Arrays of second level interrupt handlers for each pin of each enabled
 * I/O port.
 */

#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTA
static struct ioport_level2_s g_ioporta_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTB
static struct ioport_level2_s g_ioportb_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTC
static struct ioport_level2_s g_ioportc_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTD
static struct ioport_level2_s g_ioportd_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTE
static struct ioport_level2_s g_ioporte_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTF
static struct ioport_level2_s g_ioportf_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTG
static struct ioport_level2_s g_ioportg_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTH
static struct ioport_level2_s g_ioporth_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTJ
static struct ioport_level2_s g_ioportj_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTK
static struct ioport_level2_s g_ioportk_cnisrs;
#endif

/* Look-up of port to interrupt handler array */

static struct ioport_level2_s * const g_level2_handlers[CHIP_NPORTS] =
{
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTA
  [PIC32MZ_IOPORTA] = &g_ioporta_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTB
  [PIC32MZ_IOPORTB] = &g_ioportb_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTC
  [PIC32MZ_IOPORTC] = &g_ioportc_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTD
  [PIC32MZ_IOPORTD] = &g_ioportd_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTE
  [PIC32MZ_IOPORTE] = &g_ioporte_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTF
  [PIC32MZ_IOPORTF] = &g_ioportf_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTG
  [PIC32MZ_IOPORTG] = &g_ioportg_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTH
  [PIC32MZ_IOPORTH] = &g_ioporth_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTJ
  [PIC32MZ_IOPORTJ] = &g_ioportj_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTK
  [PIC32MZ_IOPORTK] = &g_ioportk_cnisrs,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline bool pic32mz_input(pinset_t pinset)
{
  return ((pinset & GPIO_MODE_MASK) != GPIO_INPUT);
}

static inline bool pic32mz_interrupt(pinset_t pinset)
{
  return ((pinset & GPIO_INTERRUPT) != 0);
}

static inline bool pic32mz_pullup(pinset_t pinset)
{
  return ((pinset & GPIO_PULLUP) != 0);
}

static inline bool pic32mz_pulldown(pinset_t pinset)
{
  return ((pinset & GPIO_PULLDOWN) != 0);
}

static inline unsigned int pic32mz_ioport(pinset_t pinset)
{
  return ((pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

static inline unsigned int pic32mz_pin(pinset_t pinset)
{
  return ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: pic32mz_cninterrupt
 *
 * Description:
 *  Common change notification interrupt handler.
 *
 ****************************************************************************/

static int pic32mz_cninterrupt(int irq, FAR void *context)
{
  struct ioport_level2_s *handlers;
  xcpt_t handler;
  uintptr_t base;
  uint32_t cnstat;
  uint32_t cnen;
  uint32_t pending;
  uint32_t regval;
  int ioport;
  int status;
  int ret = OK;
  int i;

  /* Get the IO port index from the IRQ number.  This, of course,
   * assumes that the irq numbers are consecutive beginning with
   * IOPORTA.
   */

  ioport   = irq - PIC32MZ_IRQ_PORTA;
  DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

  /* If we got this interrupt, then there must also be an array
   * of second level handlers (and a base address) for the IOPORT.
   */

  handlers = g_level2_handlers[ioport];
  base     = g_gpiobase[ioport];
  DEBUGASSERT(handlers && base);

  if (handlers && base)
    {
      /* Get the interrupt status associated with this interrupt */

      cnstat  = getreg32(base + PIC32MZ_IOPORT_CNSTAT_OFFSET);
      cnen    = getreg32(base + PIC32MZ_IOPORT_CNSTAT_OFFSET);
      pending = cnstat & cnen;

      /* Hmmm.. the data sheet implies that the status will pend
       * until the corresponding PORTx registers is read?  Clear
       * pending status.
       */

      regval = getreg32(base + PIC32MZ_IOPORT_PORT_OFFSET);
      UNUSED(regval);

      /* Call all attached handlers for each pending interrupt */

      for (i = 0; i < 16; i++)
        {
          /* Is this interrupt pending */

          if ((pending & (1 << IOPORT_CNSTAT(i))) != 0)
            {
              /* Yes.. Has the user attached a handler? */

              handler = handlers->handler[i];
              if (handler)
                {
                  /* Yes.. call the attached handler */

                  status = handler(irq, context);

                  /* Keep track of the status of the last handler that
                   * failed.
                   */

                  if (status < 0)
                    {
                      ret = status;
                    }
                }
            }
        }
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(irq);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a GPIO change notification interrupts.
 *   This function is called internally by the system on power up and should
 *   not be called again.
 *
 ****************************************************************************/

void pic32mz_gpioirqinitialize(void)
{
  uintptr_t base;
  int ret;
  int i;

  /* Perform initialization for each IO port that has interrupt support
   * enabled.  We can tell this because there will be an array of second
   * level handlers for each enabled IO port.
   */

  for (i = 0; i < CHIP_NPORTS; i++)
    {
      /* Get the base address of this IO port peripheral */

      base = g_gpiobase[i];
      DEBUGASSERT(base);

      /* Reset all registers and disable the CN module */

      putreg32(IOPORT_CNEN_ALL, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);
      putreg32(IOPORT_CNPU_ALL, base + PIC32MZ_IOPORT_CNPUCLR_OFFSET);
      putreg32(IOPORT_CNPD_ALL, base + PIC32MZ_IOPORT_CNPDCLR_OFFSET);
      putreg32(0,               base + PIC32MZ_IOPORT_CNCON_OFFSET);

      /* Is interrupt support selected for this IO port? */

      if (g_level2_handlers[i] != NULL)
        {
          /* Yes.. Attach the common change notice interrupt handler
           * to the IO port interrupt.  Notice that this assumes that
           * each IRQ number is consecutive beginning with IOPORTA.
           */

          ret = irq_attach(PIC32MZ_IRQ_PORTA + i, pic32mz_cninterrupt);
          DEBUGASSERT(ret == OK);
          UNUSED(ret);

          /* Enable the CN module.  NOTE that the CN module is active when
           * in sleep mode.
           */

          putreg32(IOPORT_CNCON_ON, base + PIC32MZ_IOPORT_CNCON_OFFSET);

          /* And enable the GPIO interrupt.  Same assumption as above. */

          up_enable_irq(PIC32MZ_IRQ_PORTA + i);
        }
    }
}

/****************************************************************************
 * Name: pic32mz_gpioattach
 *
 * Description:
 *   Attach an interrupt service routine to a GPIO interrupt.  This will
 *   also reconfigure the pin as an interrupting input.  The change
 *   notification number is associated with all interrupt-capabile GPIO pins.
 *   The association could, however, differ from part to part and must be
 *   provided by the caller.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin.
 *   In that case, all attached handlers will be called.  Each handler must
 *   maintain state and determine if the underlying GPIO input value changed.
 *
 * Parameters:
 *  - pinset:  GPIO pin configuration
 *  - pin:      The change notification number associated with the pin.
 *  - handler: Interrupt handler (may be NULL to detach)
 *
 * Returns:
 *  The previous value of the interrupt handler function pointer.  This
 *  value may, for example, be used to restore the previous handler when
 *  multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t pic32mz_gpioattach(pinset_t pinset, xcpt_t handler)
{
  struct ioport_level2_s *handlers;
  xcpt_t oldhandler = NULL;
  irqstate_t flags;
  uintptr_t base;
  int ioport;
  int pin;

  DEBUGASSERT(pin < IOPORT_NUMCN);

  /* First verify that the pinset is configured as an interrupting input */

  if (pic32mz_input(pinset) && pic32mz_interrupt(pinset))
    {
      /* Get the ioport index and pin number from the pinset */

      ioport = pic32mz_ioport(pinset);
      pin    = pic32mz_pin(pinset);
      DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

      /* Get the register base address for this port */

      base = g_gpiobase[ioport];
      DEBUGASSERT(base);

      /* If this IO port has been properly configured for interrupts, then
       * there should be an array of second level interrupt handlers
       * allocated for it.
       */

      handlers = g_level2_handlers[ioport];
      DEBUGASSERT(handlers);
      if (handlers)
        {
          /* Get the previously attached handler as the return value */

          flags = enter_critical_section();
          oldhandler = handlers->handler[pin];

          /* Are we attaching or detaching? */

          if (handler != NULL)
            {
              /* Attaching... Make sure that the GPIO is properly configured
               * as an input
               */

              pic32mz_configgpio(pinset);

              /* Pull-up requested? */

              if (pic32mz_pullup(pinset))
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPUSET_OFFSET);
                }
              else
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPUCLR_OFFSET);
                }

              /* Pull-down requested? */

              if (pic32mz_pulldown(pinset))
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPDSET_OFFSET);
                }
              else
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPDCLR_OFFSET);
                }
            }
          else
            {
              /* Disable the pull-up/downs (just so things look better in
               * the debugger).
               */

               putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPUCLR_OFFSET);
               putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPDCLR_OFFSET);
            }

          /* Whether attaching or detaching, the next state of the interrupt
           * should be disabled.
           */

          putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);

          /* Set the new handler (perhaps NULLifying the current handler) */

          handlers->handler[pin] = handler;
          leave_critical_section(flags);
        }
    }

  return oldhandler;
}

/****************************************************************************
 * Name: pic32mz_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mz_gpioirqenable(pinset_t pinset)
{
  uintptr_t base;
  int ioport;
  int pin;

  /* Get the ioport index and pin number from the pinset */

  ioport = pic32mz_ioport(pinset);
  pin    = pic32mz_pin(pinset);
  DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

  /* Get the register base address for this port */

  base = g_gpiobase[ioport];
  DEBUGASSERT(base);
  if (base)
    {
      /* And enable the interrupt.  NOTE that we don't actually check if
       * interrupts are configured for this IO port.  If not, this operation
       * should do nothing.
       */

      putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENSET_OFFSET);
    }
}

/****************************************************************************
 * Name: pic32mz_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mz_gpioirqdisable(pinset_t pinset)
{
  uintptr_t base;
  int ioport;
  int pin;

  /* Get the ioport index and pin number from the pinset */

  ioport = pic32mz_ioport(pinset);
  pin    = pic32mz_pin(pinset);
  DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

  /* Get the register base address for this port */

  base = g_gpiobase[ioport];
  DEBUGASSERT(base);
  if (base)
    {
      /* And disable the interrupt.  NOTE that we don't actually check if
       * interrupts are configured for this IO port.  If not, this operation
       * should do nothing.
       */

      putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);
    }
}

#endif /* CONFIG_PIC32MZ_GPIOIRQ */
