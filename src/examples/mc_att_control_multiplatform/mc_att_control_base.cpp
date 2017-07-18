/* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
 * @file mc_att_control_base.cpp
 *
 * MC Attitude Controller : Control and math code
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Roman Bapst <bapstr@ethz.ch>
 *
 */

#include "mc_att_control_base.h"
#include <geo/geo.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>

#ifdef CONFIG_ARCH_ARM
#else
#include <cmath>
using namespace std;
#endif

MulticopterAttitudeControlBase::MulticopterAttitudeControlBase() :
	_v_rates_sp_mod(),
	_actuators(),
	_publish_att_sp(false)

{
	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.yaw_ff = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.acro_rate_max.zero();

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();
}

MulticopterAttitudeControlBase::~MulticopterAttitudeControlBase()
{
}

void MulticopterAttitudeControlBase::control_attitude(float dt)
{
	_thrust_sp = _v_att_sp->data().thrust;

	/* construct attitude setpoint rotation matrix */
	math::Quaternion q_sp(&_v_att_sp->data().q_d[0]);
	math::Matrix<3, 3> R_sp = q_sp.to_dcm();

	/* rotation matrix for current state */
	math::Quaternion q(&_v_att->data().q[0]);
	math::Matrix<3, 3> R = q.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector <3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector <3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector <3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector <3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector <3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector <3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion ql;
		ql.from_dcm(R.transposed() * R_sp);
		math::Vector <3> e_R_d = ql.imag();
		e_R_d.normalize();
		e_R_d *= 2.0f * atan2f(e_R_d.length(), ql(0));

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* limit yaw rate */
	_rates_sp(2) = math::constrain(_rates_sp(2), -_params.yaw_rate_max,
				       _params.yaw_rate_max);

	/* feed forward yaw setpoint rate */
	_rates_sp(2) += _v_att_sp->data().yaw_sp_move_rate * yaw_w * _params.yaw_ff;

}

void MulticopterAttitudeControlBase::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed->data().armed || !_v_status->data().is_rotary_wing) {
		_rates_int.zero();
	}

	/* current body angular rates */
	math::Vector < 3 > rates;
	rates(0) = _v_att->data().rollspeed;
	rates(1) = _v_att->data().pitchspeed;
	rates(2) = _v_att->data().yawspeed;

	/* angular rates error */
	math::Vector < 3 > rates_err = _rates_sp - rates;
	_att_control = _params.rate_p.emult(rates_err)
		       + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int;
	_rates_prev = rates;

	/* update integral only if not saturated on low limit */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
				float rate_i = _rates_int(i)
					       + _params.rate_i(i) * rates_err(i) * dt;

				if (isfinite(
					    rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}

}

void MulticopterAttitudeControlBase::set_actuator_controls()
{
	_actuators.data().control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
	_actuators.data().control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
	_actuators.data().control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
	_actuators.data().control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
}
