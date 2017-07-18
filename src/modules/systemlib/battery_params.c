/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file battery_params.c
 *
 * Parameters defined by the battery lib.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Empty cell voltage (5C load)
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen before the steep dropoff to 2.8V. A typical
 * lithium battery can only be discharged down to 10% before it drops off
 * to a voltage level damaging the cells.
 *
 * @group Battery Calibration
 * @unit V
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_V_EMPTY, 3.4f);

/**
 * Full cell voltage (5C load)
 *
 * Defines the voltage where a single cell of the battery is considered full
 * under a mild load. This will never be the nominal voltage of 4.2V
 *
 * @group Battery Calibration
 * @unit V
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_V_CHARGED, 4.05f);

/**
 * Low threshold
 *
 * Sets the threshold when the battery will be reported as low.
 * This has to be higher than the critical threshold.
 *
 * @group Battery Calibration
 * @unit norm
 * @min 0.12
 * @max 0.4
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_LOW_THR, 0.15f);

/**
 * Critical threshold
 *
 * Sets the threshold when the battery will be reported as critically low.
 * This has to be lower than the low threshold. This threshold commonly
 * will trigger RTL.
 *
 * @group Battery Calibration
 * @unit norm
 * @min 0.05
 * @max 0.1
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_CRIT_THR, 0.07f);

/**
 * Emergency threshold
 *
 * Sets the threshold when the battery will be reported as dangerously low.
 * This has to be lower than the critical threshold. This threshold commonly
 * will trigger landing.
 *
 * @group Battery Calibration
 * @unit norm
 * @min 0.03
 * @max 0.07
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_EMERGEN_THR, 0.05f);

/**
 * Voltage drop per cell on full throttle
 *
 * This implicitely defines the internal resistance
 * to maximum current ratio and assumes linearity.
 * A good value to use is the difference between the
 * 5C and 20-25C load. Not used if BAT_R_INTERNAL is
 * set.
 *
 * @group Battery Calibration
 * @unit V
 * @min 0.07
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_V_LOAD_DROP, 0.3f);

/**
 * Explicitly defines the per cell internal resistance
 *
 * If non-negative, then this will be used in place of
 * BAT_V_LOAD_DROP for all calculations.
 *
 * @group Battery Calibration
 * @unit Ohms
 * @min -1.0
 * @max 0.2
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_R_INTERNAL, -1.0f);

/**
 * Number of cells.
 *
 * Defines the number of cells the attached battery consists of.
 *
 * @group Battery Calibration
 * @unit S
 * @value 0 Unconfigured
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 * @reboot_required true
 */
PARAM_DEFINE_INT32(BAT_N_CELLS, 0);

/**
 * Battery capacity.
 *
 * Defines the capacity of the attached battery.
 *
 * @group Battery Calibration
 * @unit mA
 * @decimal 0
 * @min -1.0
 * @max 100000
 * @increment 50
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_CAPACITY, -1.0f);
