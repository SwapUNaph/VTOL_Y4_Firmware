/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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
 * @file integrator.h
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <mathlib/mathlib.h>

class Integrator
{
public:
	Integrator(uint64_t auto_reset_interval = 4000 /* 250 Hz */, bool coning_compensation = false);
	virtual ~Integrator();

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value.
	 * @param val		Item to put.
	 * @param integral	Current integral in case the integrator did reset, else the value will not be modified
	 * @param integral_dt	Get the dt in us of the current integration (only if reset).
	 * @return		true if putting the item triggered an integral reset and the integral should be
	 *			published.
	 */
	bool put(uint64_t timestamp, math::Vector<3> &val, math::Vector<3> &integral, uint64_t &integral_dt);

	/**
	 * Put an item into the integral but provide an interval instead of a timestamp.
	 *
	 * @param interval_us	Interval in us since last integration.
	 * @param val		Item to put.
	 * @param integral	Current integral in case the integrator did reset, else the value will not be modified
	 * @param integral_dt	Get the dt in us of the current integration (only if reset). Note that this
	 *			values might not be accurate vs. hrt_absolute_time because it is just the sum of the
	 *			supplied intervals.
	 * @return		true if putting the item triggered an integral reset and the integral should be
	 *			published.
	 */
	bool put_with_interval(unsigned interval_us, math::Vector<3> &val, math::Vector<3> &integral,
			       uint64_t &integral_dt);

	/**
	 * Get the current integral and reset the integrator if needed.
	 *
	 * @param reset	    	Reset the integral to zero.
	 * @param integral_dt	Get the dt in us of the current integration (only if reset).
	 * @return		the integral since the last read-reset
	 */
	math::Vector<3>		get(bool reset, uint64_t &integral_dt);

	/**
	 * Get the current integral and reset the integrator if needed. Additionally give the
	 * integral over the samples differentiated by the integration time (mean filtered values).
	 *
	 * @param reset	    	Reset the integral to zero.
	 * @param integral_dt	Get the dt in us of the current integration (only if reset).
	 * @param filtered_val	The integral differentiated by the integration time.
	 * @return		the integral since the last read-reset
	 */
	math::Vector<3>		get_and_filtered(bool reset, uint64_t &integral_dt, math::Vector<3> &filtered_val);

private:
	uint64_t _auto_reset_interval;			/**< the interval after which the content will be published
							     and the integrator reset, 0 if no auto-reset */
	uint64_t _last_integration_time;		/**< timestamp of the last integration step */
	uint64_t _last_reset_time;			/**< last auto-announcement of integral value */
	math::Vector<3> _alpha;				/**< integrated value before coning corrections are applied */
	math::Vector<3> _last_alpha;			/**< previous value of _alpha */
	math::Vector<3> _beta;				/**< accumulated coning corrections */
	math::Vector<3> _last_val;			/**< previous input */
	math::Vector<3> _last_delta_alpha;		/**< integral from previous previous sampling interval */
	bool _coning_comp_on;				/**< true to turn on coning corrections */

	/* we don't want this class to be copied */
	Integrator(const Integrator &);
	Integrator operator=(const Integrator &);

	/* Do a reset.
	 *
	 * @param integral_dt	Get the dt in us of the current integration.
	 */
	void _reset(uint64_t &integral_dt);
};
