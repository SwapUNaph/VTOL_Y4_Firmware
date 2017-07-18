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
* @file tiltrotor.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
*
*/

#ifndef TILTROTOR_H
#define TILTROTOR_H
#include "vtol_type.h"
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>

class Tiltrotor : public VtolType
{

public:

	Tiltrotor(VtolAttitudeControl *_att_controller);
	~Tiltrotor();

	virtual void update_vtol_state();
	virtual void update_transition_state();
	virtual void fill_actuator_outputs();
	virtual void update_mc_state();
	virtual void update_fw_state();
	virtual void waiting_on_tecs();

private:

	struct {
		float front_trans_dur;			/**< duration of first part of front transition */
		float back_trans_dur;			/**< duration of back transition */
		float tilt_mc;					/**< actuator value corresponding to mc tilt */
		float tilt_transition;			/**< actuator value corresponding to transition tilt (e.g 45 degrees) */
		float tilt_fw;					/**< actuator value corresponding to fw tilt */
		float airspeed_trans;			/**< airspeed at which we switch to fw mode after transition */
		float airspeed_blend_start;		/**< airspeed at which we start blending mc/fw controls */
		int elevons_mc_lock;			/**< lock elevons in multicopter mode */
		float front_trans_dur_p2;
		int fw_motors_off;			/**< bitmask of all motors that should be off in fixed wing mode */
		int airspeed_mode;
		int diff_thrust;
		float diff_thrust_scale;
	} _params_tiltrotor;

	struct {
		param_t front_trans_dur;
		param_t back_trans_dur;
		param_t tilt_mc;
		param_t tilt_transition;
		param_t tilt_fw;
		param_t airspeed_trans;
		param_t airspeed_blend_start;
		param_t elevons_mc_lock;
		param_t front_trans_dur_p2;
		param_t fw_motors_off;
		param_t airspeed_mode;
		param_t diff_thrust;
		param_t diff_thrust_scale;
	} _params_handles_tiltrotor;

	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	/**
	 * Specific to tiltrotor with vertical aligned rear engine/s.
	 * These engines need to be shut down in fw mode. During the back-transition
	 * they need to idle otherwise they need too much time to spin up for mc mode.
	 */
	enum rear_motor_state {
		ENABLED = 0,
		DISABLED,
		IDLE,
		VALUE
	} _rear_motors;

	struct {
		vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

	float _tilt_control;		/**< actuator value for the tilt servo */

	const float _min_front_trans_dur;	/**< min possible time in which rotors are rotated into the first position */

	/**
	 * Return a bitmap of channels that should be turned off in fixed wing mode.
	 */
	int get_motor_off_channels(const int channels);

	/**
	 * Return true if the motor channel is off in fixed wing mode.
	 */
	bool is_motor_off_channel(const int channel);

	/**
	 * Adjust the state of the rear motors. In fw mode they shouldn't spin.
	 */
	void set_rear_motor_state(rear_motor_state state, int value = 0);

	/**
	 * Update parameters.
	 */
	virtual void parameters_update();

};
#endif
