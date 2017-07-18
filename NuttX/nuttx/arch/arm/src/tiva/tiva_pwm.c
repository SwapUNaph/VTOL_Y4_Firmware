/************************************************************************************
 * arch/arm/src/tiva/tiva_pwm.c
 *
 *   Copyright (C) 2016 Young Mu. All rights reserved.
 *   Author: Young Mu <young.mu@aliyun.com>
 *
 * The basic structure of this driver derives in spirit (if nothing more) from the
 * NuttX SAM PWM driver which has:
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/drivers/pwm.h>

#include "up_arch.h"
#include "tiva_gpio.h"
#include "tiva_pwm.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"

#include "chip/tiva_pwm.h"
#include "chip/tiva_pinmap.h"
#include "chip/tm4c_memorymap.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/

uint32_t g_pwm_pinset[] =
{
  GPIO_M0_PWM0,
  GPIO_M0_PWM1,
  GPIO_M0_PWM2,
  GPIO_M0_PWM3,
  GPIO_M0_PWM4,
  GPIO_M0_PWM5,
  GPIO_M0_PWM6,
  GPIO_M0_PWM7,
};

struct tiva_pwm_chan_s
{
  const struct pwm_ops_s *ops;
  uint8_t controller_id;
  uintptr_t controller_base;
  uint8_t generator_id;
  uintptr_t generator_base;
  uint8_t channel_id;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline void tiva_pwm_putreg(struct tiva_pwm_chan_s *chan,
                                   unsigned int offset, uint32_t regval);
static inline uint32_t tiva_pwm_getreg(struct tiva_pwm_chan_s *chan,
                                       unsigned int offset);

static int tiva_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int tiva_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int tiva_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                          FAR const struct pwm_info_s *info);
static int tiva_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int tiva_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                          int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static uint32_t g_pwm_freq = 15000000;
static uint32_t g_pwm_counter = (1 << 16);

static const struct pwm_ops_s g_pwm_ops =
{
  .setup    = tiva_pwm_setup,
  .shutdown = tiva_pwm_shutdown,
  .start    = tiva_pwm_start,
  .stop     = tiva_pwm_stop,
  .ioctl    = tiva_pwm_ioctl,
};

#ifdef CONFIG_TIVA_PWM0_CHAN0
static struct tiva_pwm_chan_s g_pwm_chan0 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 0,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 0,
  .channel_id      = 0,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN1
static struct tiva_pwm_chan_s g_pwm_chan1 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 0,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 0,
  .channel_id      = 1,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN2
static struct tiva_pwm_chan_s g_pwm_chan2 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 1,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 1,
  .channel_id      = 2,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN3
static struct tiva_pwm_chan_s g_pwm_chan3 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 1,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 1,
  .channel_id      = 3,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN4
static struct tiva_pwm_chan_s g_pwm_chan4 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 2,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 2,
  .channel_id      = 4,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN5
static struct tiva_pwm_chan_s g_pwm_chan5 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 2,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 2,
  .channel_id      = 5,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN6
static struct tiva_pwm_chan_s g_pwm_chan6 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id = 3,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 3,
  .channel_id      = 6,
};
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN7
static struct tiva_pwm_chan_s g_pwm_chan7 =
{
  .ops             = &g_pwm_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 3,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMn_BASE + TIVA_PWMn_INTERVAL * 3,
  .channel_id      = 7,
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: tiva_pwm_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ************************************************************************************/

static inline uint32_t tiva_pwm_getreg(struct tiva_pwm_chan_s *chan,
                                       unsigned int offset)
{
    uintptr_t regaddr = chan->generator_base + offset;
    return getreg32(regaddr);
}

/************************************************************************************
 * Name: tiva_pwm_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ************************************************************************************/

static inline void tiva_pwm_putreg(struct tiva_pwm_chan_s *chan,
                                   unsigned int offset, uint32_t regval)
{
    uintptr_t regaddr = chan->generator_base + offset;
    putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   will be configured and initialized the device so that it is ready for
 *   use.  It will not, however, output pulses until the start method is
 *   called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct tiva_pwm_chan_s *chan = (FAR struct tiva_pwm_chan_s *)dev;
  pwminfo("setup PWM for channel %d\n", chan->channel_id);

  /* Enable GPIO port, GPIO pin type and GPIO alternate function (refer to
   * TM4C1294NC 23.4.2-4)
   */

  int ret = tiva_configgpio(g_pwm_pinset[chan->channel_id]);
  if (ret < 0)
    {
      pwmerr("ERROR: tiva_configgpio failed (%x)\n",
             g_pwm_pinset[chan->channel_id]);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct tiva_pwm_chan_s *chan = (FAR struct tiva_pwm_chan_s *)dev;
  pwminfo("shutdown PWM for channel %d\n", chan->channel_id);

  /* Remove unused-variable warning */

  (void)chan;

  /* Ensure the PWM channel has been stopped */

  tiva_pwm_stop(dev);

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                          FAR const struct pwm_info_s *info)
{
  FAR struct tiva_pwm_chan_s *chan = (FAR struct tiva_pwm_chan_s *)dev;
  pwminfo("start PWM for channel %d\n", chan->channel_id);

  uint16_t duty = info->duty;
  uint32_t frequency = info->frequency;

  /* Configure PWM countdown mode (refer to TM4C1294NC 23.4.6) */

  tiva_pwm_putreg(chan, TIVA_PWMn_CTL_OFFSET, 0);
  if (chan->channel_id % 2 == 0)
    {
      tiva_pwm_putreg(chan, TIVA_PWMn_GENA_OFFSET,
                      GENx_LOW << TIVA_PWMn_GENx_ACTCMPAD | GENx_HIGH << TIVA_PWMn_GENx_ACTLOAD);
    }
  else
    {
      tiva_pwm_putreg(chan, TIVA_PWMn_GENB_OFFSET,
                      GENx_LOW << TIVA_PWMn_GENx_ACTCMPBD | GENx_HIGH << TIVA_PWMn_GENx_ACTLOAD);
    }

  /* Set the PWM period (refer to TM4C1294NC 23.4.7) */

  uint32_t pwm_min_freq = (uint32_t)(g_pwm_freq / g_pwm_counter) + 1;
  uint32_t pwm_max_freq = g_pwm_freq;
  uint32_t load = (uint32_t)(g_pwm_freq / frequency);

  pwminfo("channel %d: load = %u (%08x)\n", chan->channel_id, load, load);

  if (load >= g_pwm_counter || load < 1)
    {
      pwmerr("ERROR: frequency should be in [%d, %d] Hz\n",
              pwm_min_freq, pwm_max_freq);
      return -ERANGE;
    }

  tiva_pwm_putreg(chan, TIVA_PWMn_LOAD_OFFSET, load - 1);

  /* Configure PWM duty (refer to TM4C1294NC 23.4.8-9)
   *
   * Workaround:
   *   When comp equals to load, the signal is never pulled down,
   *   so let comp equals to (comp-1)
   */

  uint32_t comp = (uint32_t)((1 - (float)duty / g_pwm_counter) * load);
  comp = (duty == 0) ? (comp - 1) : (comp);
  pwminfo("channel %d: comp = %u (%08x)\n", chan->channel_id, comp, comp);

  if (chan->channel_id % 2 == 0)
    {
      tiva_pwm_putreg(chan, TIVA_PWMn_CMPA_OFFSET, comp - 1);
    }
  else
    {
      tiva_pwm_putreg(chan, TIVA_PWMn_CMPB_OFFSET, comp - 1);
    }

  /* Enable the PWM generator (refer to TM4C1294NC 23.4.10) */

  tiva_pwm_putreg(chan, TIVA_PWMn_CTL_OFFSET, CTL_ENABLE << TIVA_PWMn_CTL_ENABLE);

  /* Enable PWM channel (refer to TM4C1294NC 23.4.11) */

  uint32_t enable = getreg32(chan->controller_base + TIVA_PWM_ENABLE_OFFSET);
  enable |= (1 << chan->channel_id);
  putreg32(enable, chan->controller_base + TIVA_PWM_ENABLE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/

static int tiva_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct tiva_pwm_chan_s *chan = (FAR struct tiva_pwm_chan_s *)dev;
  pwminfo("stop PWM for channel %d\n", chan->channel_id);

  /* Disable PWM channel */

  uint32_t value = getreg32(chan->controller_base + TIVA_PWM_ENABLE_OFFSET);
  value &= ~(1 << chan->channel_id);
  putreg32(value, chan->controller_base + TIVA_PWM_ENABLE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  FAR struct tiva_pwm_chan_s *chan = (FAR struct tiva_pwm_chan_s *)dev;
  pwminfo("ioctl PWM for channel %d\n", chan->channel_id);

  /* Remove unused-variable warning */

  (void)chan;

  /* There are no platform-specific ioctl commands */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_pwm_initialize
 *
 * Description:
 *   Initialize one PWM channel for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel use.
 *
 * Returned Value:
 *   On success, a pointer to the SAMA5 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *tiva_pwm_initialize(int channel)
{
  assert(channel >= 0 && channel <= 7);
  FAR struct tiva_pwm_chan_s *chan;

  switch (channel)
    {
#ifdef CONFIG_TIVA_PWM0_CHAN0
    case 0:
      chan = &g_pwm_chan0;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN1
    case 1:
      chan = &g_pwm_chan1;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN2
    case 2:
      chan = &g_pwm_chan2;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN3
    case 3:
      chan = &g_pwm_chan3;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN4
    case 4:
      chan = &g_pwm_chan4;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN5
    case 5:
      chan = &g_pwm_chan5;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN6
    case 6:
      chan = &g_pwm_chan6;
      break;
#endif

#ifdef CONFIG_TIVA_PWM0_CHAN7
    case 7:
      chan = &g_pwm_chan7;
      break;
#endif

    default:
      pwmerr("ERROR: invalid channel %d\n", channel);
      return NULL;
    }

  pwminfo("channel %d: channel_id=%d, ", channel, chan->channel_id);
  pwminfo("controller_id=%d, controller_base=%08x, ",
          chan->controller_id, chan->controller_base);
  pwminfo("generator_id=%d, generator_base=%08x\n",
          chan->generator_id, chan->generator_base);

  /* Enable PWM controller (refer to TM4C1294NC 23.4.1) */

  assert(chan->controller_id == 0);
  tiva_pwm_enablepwr(chan->controller_id);
  tiva_pwm_enableclk(chan->controller_id);

  /* Configure PWM Clock Configuration (refer to TM4C1294NC 23.4.5)
   *
   * On TM4C1294NC, configure the PWM clock source as 15MHz (the system
   * clock 120MHz divided by 8)
   *
   * TODO: need an algorithm to choose the best divider and load value combo.
   */

  putreg32(CC_USEPWM << TIVA_PWM_CC_USEPWM | CC_PWMDIV_8 << TIVA_PWM_CC_PWMDIV,
           chan->controller_base + TIVA_PWM_CC);

  return (FAR struct pwm_lowerhalf_s *)chan;
}
