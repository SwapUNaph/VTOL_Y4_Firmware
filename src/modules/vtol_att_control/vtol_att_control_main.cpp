/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover,forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */
#include "vtol_att_control_main.h"
#include <systemlib/mavlink_log.h>

namespace VTOL_att_control
{
VtolAttitudeControl *g_control;
}

/**
* Constructor
*/
VtolAttitudeControl::VtolAttitudeControl() :
	_task_should_exit(false),
	_control_task(-1),

	// mavlink log
	_mavlink_log_pub(nullptr),

	//init subscription handlers
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_mc_virtual_att_sp_sub(-1),
	_fw_virtual_att_sp_sub(-1),
	_mc_virtual_v_rates_sp_sub(-1),
	_fw_virtual_v_rates_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_local_pos_sub(-1),
	_airspeed_sub(-1),
	_battery_status_sub(-1),
	_vehicle_cmd_sub(-1),
	_tecs_status_sub(-1),
	_land_detected_sub(-1),

	//init publication handlers
	_actuators_0_pub(nullptr),
	_actuators_1_pub(nullptr),
	_vtol_vehicle_status_pub(nullptr),
	_v_rates_sp_pub(nullptr),
	_v_att_sp_pub(nullptr),
	_transition_command(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
	_abort_front_transition(false)

{
	memset(& _vtol_vehicle_status, 0, sizeof(_vtol_vehicle_status));
	_vtol_vehicle_status.vtol_in_rw_mode = true;	/* start vtol in rotary wing mode*/
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_mc_virtual_att_sp, 0, sizeof(_mc_virtual_att_sp));
	memset(&_fw_virtual_att_sp, 0, sizeof(_fw_virtual_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_mc_virtual_v_rates_sp, 0, sizeof(_mc_virtual_v_rates_sp));
	memset(&_fw_virtual_v_rates_sp, 0, sizeof(_fw_virtual_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators_out_0, 0, sizeof(_actuators_out_0));
	memset(&_actuators_out_1, 0, sizeof(_actuators_out_1));
	memset(&_actuators_mc_in, 0, sizeof(_actuators_mc_in));
	memset(&_actuators_fw_in, 0, sizeof(_actuators_fw_in));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_airspeed, 0, sizeof(_airspeed));
	memset(&_batt_status, 0, sizeof(_batt_status));
	memset(&_vehicle_cmd, 0, sizeof(_vehicle_cmd));
	memset(&_tecs_status, 0, sizeof(_tecs_status));
	memset(&_land_detected, 0, sizeof(_land_detected));

	_params.idle_pwm_mc = PWM_DEFAULT_MIN;
	_params.vtol_motor_count = 0;
	_params.vtol_fw_permanent_stab = 0;

	_params_handles.idle_pwm_mc = param_find("VT_IDLE_PWM_MC");
	_params_handles.vtol_motor_count = param_find("VT_MOT_COUNT");
	_params_handles.vtol_fw_permanent_stab = param_find("VT_FW_PERM_STAB");
	_params_handles.mc_airspeed_min = param_find("VT_MC_ARSPD_MIN");
	_params_handles.mc_airspeed_max = param_find("VT_MC_ARSPD_MAX");
	_params_handles.mc_airspeed_trim = param_find("VT_MC_ARSPD_TRIM");
	_params_handles.fw_pitch_trim = param_find("VT_FW_PITCH_TRIM");
	_params_handles.power_max = param_find("VT_POWER_MAX");
	_params_handles.prop_eff = param_find("VT_PROP_EFF");
	_params_handles.arsp_lp_gain = param_find("VT_ARSP_LP_GAIN");
	_params_handles.vtol_type = param_find("VT_TYPE");
	_params_handles.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_params_handles.fw_min_alt = param_find("VT_FW_MIN_ALT");
	_params_handles.fw_qc_max_pitch = param_find("VT_FW_QC_P");
	_params_handles.fw_qc_max_roll = param_find("VT_FW_QC_R");
	_params_handles.front_trans_time_openloop = param_find("VT_F_TR_OL_TM");
	_params_handles.front_trans_time_min = param_find("VT_TRANS_MIN_TM");

	/* fetch initial parameter values */
	parameters_update();

	if (_params.vtol_type == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);

	} else if (_params.vtol_type == vtol_type::TILTROTOR) {
		_vtol_type = new Tiltrotor(this);

	} else if (_params.vtol_type == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);

	} else {
		_task_should_exit = true;
	}
}

/**
* Destructor
*/
VtolAttitudeControl::~VtolAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	// free memory used by instances of base class VtolType
	if (_vtol_type != nullptr) {
		delete _vtol_type;
	}

	VTOL_att_control::g_control = nullptr;
}

/**
* Check for changes in vehicle control mode.
*/
void VtolAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

/**
* Check for changes in manual inputs.
*/
void VtolAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}
/**
* Check for arming status updates.
*/
void VtolAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

/**
* Check for inputs from mc attitude controller.
*/
void VtolAttitudeControl::actuator_controls_mc_poll()
{
	bool updated;
	orb_check(_actuator_inputs_mc, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);
	}
}

/**
* Check for inputs from fw attitude controller.
*/
void VtolAttitudeControl::actuator_controls_fw_poll()
{
	bool updated;
	orb_check(_actuator_inputs_fw, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
	}
}

/**
* Check for attitude rates setpoint from mc attitude controller
*/
void VtolAttitudeControl::vehicle_rates_sp_mc_poll()
{
	bool updated;
	orb_check(_mc_virtual_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mc_virtual_rates_setpoint), _mc_virtual_v_rates_sp_sub, &_mc_virtual_v_rates_sp);
	}
}

/**
* Check for attitude rates setpoint from fw attitude controller
*/
void VtolAttitudeControl::vehicle_rates_sp_fw_poll()
{
	bool updated;
	orb_check(_fw_virtual_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(fw_virtual_rates_setpoint), _fw_virtual_v_rates_sp_sub, &_fw_virtual_v_rates_sp);
	}
}

/**
* Check for airspeed updates.
*/
void
VtolAttitudeControl::vehicle_airspeed_poll()
{
	bool updated;
	orb_check(_airspeed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

/**
* Check for attitude set points update.
*/
void
VtolAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

/**
* Check for attitude update.
*/
void
VtolAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

/**
* Check for battery updates.
*/
void
VtolAttitudeControl::vehicle_battery_poll()
{
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_batt_status);
	}
}

/**
* Check for parameter updates.
*/
void
VtolAttitudeControl::parameters_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

/**
* Check for sensor updates.
*/
void
VtolAttitudeControl::vehicle_local_pos_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

}

/**
* Check for mc virtual attitude setpoint updates.
*/
void
VtolAttitudeControl::mc_virtual_att_sp_poll()
{
	bool updated;

	orb_check(_mc_virtual_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mc_virtual_attitude_setpoint), _mc_virtual_att_sp_sub, &_mc_virtual_att_sp);
	}

}

/**
* Check for fw virtual attitude setpoint updates.
*/
void
VtolAttitudeControl::fw_virtual_att_sp_poll()
{
	bool updated;

	orb_check(_fw_virtual_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(fw_virtual_attitude_setpoint), _fw_virtual_att_sp_sub, &_fw_virtual_att_sp);
	}

}

/**
* Check for command updates.
*/
void
VtolAttitudeControl::vehicle_cmd_poll()
{
	bool updated;
	orb_check(_vehicle_cmd_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command), _vehicle_cmd_sub, &_vehicle_cmd);
		handle_command();
	}
}

/**
* Check for TECS status updates.
*/
void
VtolAttitudeControl::tecs_status_poll()
{
	bool updated;

	orb_check(_tecs_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tecs_status), _tecs_status_sub, &_tecs_status);
	}
}

/**
* Check for land detector updates.
*/
void
VtolAttitudeControl::land_detected_poll()
{
	bool updated;

	orb_check(_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub, &_land_detected);
	}
}

/**
* Check received command
*/
void
VtolAttitudeControl::handle_command()
{
	// update transition command if necessary
	if (_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
		_transition_command = int(_vehicle_cmd.param1 + 0.5f);
	}
}

/*
 * Returns true if fixed-wing mode is requested.
 * Changed either via switch or via command.
 */
bool
VtolAttitudeControl::is_fixed_wing_requested()
{
	bool to_fw = false;

	if (_manual_control_sp.transition_switch != manual_control_setpoint_s::SWITCH_POS_NONE &&
	    _v_control_mode.flag_control_manual_enabled) {
		to_fw = (_manual_control_sp.transition_switch == manual_control_setpoint_s::SWITCH_POS_ON);

	} else {
		// listen to transition commands if not in manual or mode switch is not mapped
		to_fw = (_transition_command == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	}

	// handle abort request
	if (_abort_front_transition) {
		if (to_fw) {
			to_fw = false;

		} else {
			// the state changed to mc mode, reset the abort request
			_abort_front_transition = false;
			_vtol_vehicle_status.vtol_transition_failsafe = false;
		}
	}

	return to_fw;
}

/*
 * Abort front transition
 */
void
VtolAttitudeControl::abort_front_transition(const char *reason)
{
	if (!_abort_front_transition) {
		mavlink_log_critical(&_mavlink_log_pub, "Abort: %s", reason);
		_abort_front_transition = true;
		_vtol_vehicle_status.vtol_transition_failsafe = true;
	}
}

/**
* Update parameters.
*/
int
VtolAttitudeControl::parameters_update()
{
	float v;
	int l;
	/* idle pwm for mc mode */
	param_get(_params_handles.idle_pwm_mc, &_params.idle_pwm_mc);

	/* vtol motor count */
	param_get(_params_handles.vtol_motor_count, &_params.vtol_motor_count);

	/* vtol fw permanent stabilization */
	param_get(_params_handles.vtol_fw_permanent_stab, &_params.vtol_fw_permanent_stab);

	/* vtol mc mode min airspeed */
	param_get(_params_handles.mc_airspeed_min, &v);
	_params.mc_airspeed_min = v;

	/* vtol mc mode max airspeed */
	param_get(_params_handles.mc_airspeed_max, &v);
	_params.mc_airspeed_max = v;

	/* vtol mc mode trim airspeed */
	param_get(_params_handles.mc_airspeed_trim, &v);
	_params.mc_airspeed_trim = v;

	/* vtol pitch trim for fw mode */
	param_get(_params_handles.fw_pitch_trim, &v);
	_params.fw_pitch_trim = v;

	/* vtol maximum power engine can produce */
	param_get(_params_handles.power_max, &v);
	_params.power_max = v;

	/* vtol propeller efficiency factor */
	param_get(_params_handles.prop_eff, &v);
	_params.prop_eff = v;

	/* vtol total airspeed estimate low pass gain */
	param_get(_params_handles.arsp_lp_gain, &v);
	_params.arsp_lp_gain = v;

	param_get(_params_handles.vtol_type, &l);
	_params.vtol_type = l;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles.elevons_mc_lock, &l);
	_params.elevons_mc_lock = l;

	/* minimum relative altitude for FW mode (QuadChute) */
	param_get(_params_handles.fw_min_alt, &v);
	_params.fw_min_alt = v;

	/* maximum pitch angle (QuadChute) */
	param_get(_params_handles.fw_qc_max_pitch, &l);
	_params.fw_qc_max_pitch = l;

	/* maximum roll angle (QuadChute) */
	param_get(_params_handles.fw_qc_max_roll, &l);
	_params.fw_qc_max_roll = l;

	param_get(_params_handles.front_trans_time_openloop, &_params.front_trans_time_openloop);

	param_get(_params_handles.front_trans_time_min, &_params.front_trans_time_min);

	/*
	 * Minimum transition time can be maximum 90 percent of the open loop transition time,
	 * anything else makes no sense and can potentially lead to numerical problems.
	 */
	_params.front_trans_time_min = math::min(_params.front_trans_time_openloop * 0.9f,
				       _params.front_trans_time_min);

	// update the parameters of the instances of base VtolType
	if (_vtol_type != nullptr) {
		_vtol_type->parameters_update();
	}

	return OK;
}

/**
* Prepare message for mc attitude rates setpoint topic
*/
void VtolAttitudeControl::fill_mc_att_rates_sp()
{
	_v_rates_sp.timestamp 	= _mc_virtual_v_rates_sp.timestamp;
	_v_rates_sp.roll 	= _mc_virtual_v_rates_sp.roll;
	_v_rates_sp.pitch 	= _mc_virtual_v_rates_sp.pitch;
	_v_rates_sp.yaw 	= _mc_virtual_v_rates_sp.yaw;
	_v_rates_sp.thrust 	= _mc_virtual_v_rates_sp.thrust;
}

/**
* Prepare message for fw attitude rates setpoint topic
*/
void VtolAttitudeControl::fill_fw_att_rates_sp()
{
	_v_rates_sp.timestamp 	= _fw_virtual_v_rates_sp.timestamp;
	_v_rates_sp.roll 	= _fw_virtual_v_rates_sp.roll;
	_v_rates_sp.pitch 	= _fw_virtual_v_rates_sp.pitch;
	_v_rates_sp.yaw 	= _fw_virtual_v_rates_sp.yaw;
	_v_rates_sp.thrust 	= _fw_virtual_v_rates_sp.thrust;
}

void VtolAttitudeControl::publish_att_sp()
{
	if (_v_att_sp_pub != nullptr) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);

	} else {
		/* advertise and publish */
		_v_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
	}
}

void
VtolAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	VTOL_att_control::g_control->task_main();
}

void VtolAttitudeControl::task_main()
{
	fflush(stdout);

	/* do subscriptions */
	_v_att_sp_sub          = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_mc_virtual_att_sp_sub = orb_subscribe(ORB_ID(mc_virtual_attitude_setpoint));
	_fw_virtual_att_sp_sub = orb_subscribe(ORB_ID(fw_virtual_attitude_setpoint));
	_mc_virtual_v_rates_sp_sub = orb_subscribe(ORB_ID(mc_virtual_rates_setpoint));
	_fw_virtual_v_rates_sp_sub = orb_subscribe(ORB_ID(fw_virtual_rates_setpoint));
	_v_att_sub             = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_att_sp_sub          = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub            = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub             = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub         = orb_subscribe(ORB_ID(vehicle_local_position));
	_airspeed_sub          = orb_subscribe(ORB_ID(airspeed));
	_battery_status_sub	   = orb_subscribe(ORB_ID(battery_status));
	_vehicle_cmd_sub	   = orb_subscribe(ORB_ID(vehicle_command));
	_tecs_status_sub = orb_subscribe(ORB_ID(tecs_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	_actuator_inputs_mc    = orb_subscribe(ORB_ID(actuator_controls_virtual_mc));
	_actuator_inputs_fw    = orb_subscribe(ORB_ID(actuator_controls_virtual_fw));

	parameters_update();  // initialize parameter cache

	/* update vtol vehicle status*/
	_vtol_vehicle_status.fw_permanent_stab = _params.vtol_fw_permanent_stab == 1 ? true : false;

	// make sure we start with idle in mc mode
	_vtol_type->set_idle_mc();

	/* wakeup source*/
	px4_pollfd_struct_t fds[3] = {};	/*input_mc, input_fw, parameters*/

	fds[0].fd     = _actuator_inputs_mc;
	fds[0].events = POLLIN;
	fds[1].fd     = _actuator_inputs_fw;
	fds[1].events = POLLIN;
	fds[2].fd     = _params_sub;
	fds[2].events = POLLIN;

	while (!_task_should_exit) {
		/*Advertise/Publish vtol vehicle status*/
		_vtol_vehicle_status.timestamp = hrt_absolute_time();

		if (_vtol_vehicle_status_pub != nullptr) {
			orb_publish(ORB_ID(vtol_vehicle_status), _vtol_vehicle_status_pub, &_vtol_vehicle_status);

		} else {
			_vtol_vehicle_status_pub = orb_advertise(ORB_ID(vtol_vehicle_status), &_vtol_vehicle_status);
		}

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);


		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[2].revents & POLLIN) {	//parameters were updated, read them now
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		_vtol_vehicle_status.fw_permanent_stab = _params.vtol_fw_permanent_stab == 1 ? true : false;

		mc_virtual_att_sp_poll();
		fw_virtual_att_sp_poll();
		vehicle_control_mode_poll();	//Check for changes in vehicle control mode.
		vehicle_manual_poll();			//Check for changes in manual inputs.
		arming_status_poll();			//Check for arming status updates.
		vehicle_attitude_setpoint_poll();//Check for changes in attitude set points
		vehicle_attitude_poll();		//Check for changes in attitude
		actuator_controls_mc_poll();	//Check for changes in mc_attitude_control output
		actuator_controls_fw_poll();	//Check for changes in fw_attitude_control output
		vehicle_rates_sp_mc_poll();
		vehicle_rates_sp_fw_poll();
		parameters_update_poll();
		vehicle_local_pos_poll();			// Check for new sensor values
		vehicle_airspeed_poll();
		vehicle_battery_poll();
		vehicle_cmd_poll();
		tecs_status_poll();
		land_detected_poll();

		// update the vtol state machine which decides which mode we are in
		_vtol_type->update_vtol_state();

		// reset transition command if not auto control
		if (_v_control_mode.flag_control_manual_enabled) {
			if (_vtol_type->get_mode() == ROTARY_WING) {
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

			} else if (_vtol_type->get_mode() == FIXED_WING) {
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

			} else if (_vtol_type->get_mode() == TRANSITION_TO_MC) {
				/* We want to make sure that a mode change (manual>auto) during the back transition
				 * doesn't result in an unsafe state. This prevents the instant fall back to
				 * fixed-wing on the switch from manual to auto */
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			}
		}

		// check in which mode we are in and call mode specific functions
		if (_vtol_type->get_mode() == ROTARY_WING) {
			// vehicle is in rotary wing mode
			_vtol_vehicle_status.vtol_in_rw_mode = true;
			_vtol_vehicle_status.vtol_in_trans_mode = false;

			// got data from mc attitude controller
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);

				_vtol_type->update_mc_state();

				fill_mc_att_rates_sp();
			}

		} else if (_vtol_type->get_mode() == FIXED_WING) {
			// vehicle is in fw mode
			_vtol_vehicle_status.vtol_in_rw_mode = false;
			_vtol_vehicle_status.vtol_in_trans_mode = false;

			// got data from fw attitude controller
			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
				vehicle_manual_poll();

				_vtol_type->update_fw_state();

				fill_fw_att_rates_sp();
			}

		} else if (_vtol_type->get_mode() == TRANSITION_TO_MC || _vtol_type->get_mode() == TRANSITION_TO_FW) {
			// vehicle is doing a transition
			_vtol_vehicle_status.vtol_in_trans_mode = true;
			_vtol_vehicle_status.vtol_in_rw_mode = true; //making mc attitude controller work during transition
			_vtol_vehicle_status.in_transition_to_fw = (_vtol_type->get_mode() == TRANSITION_TO_FW);

			bool got_new_data = false;

			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);
				got_new_data = true;
			}

			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
				got_new_data = true;
			}

			// update transition state if got any new data
			if (got_new_data) {
				_vtol_type->update_transition_state();
				fill_mc_att_rates_sp();
			}

		} else if (_vtol_type->get_mode() == EXTERNAL) {
			// we are using external module to generate attitude/thrust setpoint
			_vtol_type->update_external_state();
		}

		publish_att_sp();
		_vtol_type->fill_actuator_outputs();

		/* Only publish if the proper mode(s) are enabled */
		if (_v_control_mode.flag_control_attitude_enabled ||
		    _v_control_mode.flag_control_rates_enabled ||
		    _v_control_mode.flag_control_manual_enabled) {
			if (_actuators_0_pub != nullptr) {
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators_out_0);

			} else {
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators_out_0);
			}

			if (_actuators_1_pub != nullptr) {
				orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_out_1);

			} else {
				_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_out_1);
			}
		}

		// publish the attitude rates setpoint
		if (_v_rates_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

		} else {
			_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
		}
	}

	warnx("exit");
	_control_task = -1;
}

int
VtolAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("vtol_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 10,
					   1200,
					   (px4_main_t)&VtolAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}


int vtol_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: vtol_att_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (VTOL_att_control::g_control != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		VTOL_att_control::g_control = new VtolAttitudeControl;

		if (VTOL_att_control::g_control == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != VTOL_att_control::g_control->start()) {
			delete VTOL_att_control::g_control;
			VTOL_att_control::g_control = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (VTOL_att_control::g_control == nullptr) {
			PX4_WARN("not running");
			return 0;
		}

		delete VTOL_att_control::g_control;
		VTOL_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (VTOL_att_control::g_control) {
			PX4_WARN("running");

		} else {
			PX4_WARN("not running");
		}

		return 0;
	}

	PX4_WARN("unrecognized command");
	return 1;
}
