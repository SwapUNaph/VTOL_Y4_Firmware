/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file px4io_params.c
 *
 * Parameters defined by the PX4IO driver
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Invert direction of main output channel 1
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV1, 0);

/**
 * Invert direction of main output channel 2
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV2, 0);

/**
 * Invert direction of main output channel 3
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV3, 0);

/**
 * Invert direction of main output channel 4
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV4, 0);

/**
 * Invert direction of main output channel 5
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV5, 0);

/**
 * Invert direction of main output channel 6
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV6, 0);

/**
 * Invert direction of main output channel 7
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV7, 0);

/**
 * Invert direction of main output channel 8
 *
 * Set to 1 to invert the channel, 0 for default direction.
 *
 * @reboot_required true
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV8, 0);

/**
 * Trim value for main output channel 1
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM1, 0);

/**
 * Trim value for main output channel 2
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM2, 0);

/**
 * Trim value for main output channel 3
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM3, 0);

/**
 * Trim value for main output channel 4
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM4, 0);

/**
 * Trim value for main output channel 5
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM5, 0);

/**
 * Trim value for main output channel 6
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM6, 0);

/**
 * Trim value for main output channel 7
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM7, 0);

/**
 * Trim value for main output channel 8
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM8, 0);

/**
 * S.BUS out
 *
 * Set to 1 to enable S.BUS version 1 output instead of RSSI.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_SBUS_MODE, 0);
