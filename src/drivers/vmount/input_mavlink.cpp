/****************************************************************************
*
*   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file input_mavlink.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input_mavlink.h"
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <drivers/drv_hrt.h>

#include <px4_defines.h>
#include <px4_posix.h>
#include <errno.h>


namespace vmount
{


InputMavlinkROI::InputMavlinkROI()
{
}

InputMavlinkROI::~InputMavlinkROI()
{
	if (_vehicle_roi_sub >= 0) {
		orb_unsubscribe(_vehicle_roi_sub);
	}

	if (_position_setpoint_triplet_sub >= 0) {
		orb_unsubscribe(_position_setpoint_triplet_sub);
	}
}

int InputMavlinkROI::initialize()
{
	_vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));

	if (_vehicle_roi_sub < 0) {
		return -errno;
	}

	_position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	if (_position_setpoint_triplet_sub < 0) {
		return -errno;
	}

	return 0;
}



int InputMavlinkROI::update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	// already_active is unused, we don't care what happened previously.

	// Default to no change, set if we receive anything.
	*control_data = nullptr;

	const int num_poll = 2;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = 		_vehicle_roi_sub;
	polls[0].events = 	POLLIN;
	polls[1].fd = 		_position_setpoint_triplet_sub;
	polls[1].events = 	POLLIN;

	int ret = px4_poll(polls, num_poll, timeout_ms);

	if (ret < 0) {
		return -errno;
	}

	if (ret == 0) {
		// Timeout, _control_data is already null

	} else {
		if (polls[0].revents & POLLIN) {
			vehicle_roi_s vehicle_roi;
			orb_copy(ORB_ID(vehicle_roi), _vehicle_roi_sub, &vehicle_roi);

			_control_data.gimbal_shutter_retract = false;

			if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_NONE) {

				_control_data.type = ControlData::Type::Neutral;
				*control_data = &_control_data;

			} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_WPNEXT) {
				_read_control_data_from_position_setpoint_sub();
				_control_data.type_data.lonlat.roll_angle = 0.f;
				_control_data.type_data.lonlat.pitch_fixed_angle = -10.f;

				*control_data = &_control_data;

			} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_WPINDEX) {
				//TODO how to do this?

			} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_LOCATION) {
				control_data_set_lon_lat(vehicle_roi.lon, vehicle_roi.lat, vehicle_roi.alt);

				*control_data = &_control_data;

			} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_TARGET) {
				//TODO is this even suported?
			}

			_cur_roi_mode = vehicle_roi.mode;

			//set all other control data fields to defaults
			for (int i = 0; i < 3; ++i) {
				_control_data.stabilize_axis[i] = false;
			}
		}

		// check whether the position setpoint got updated
		if ((polls[1].revents & POLLIN) && _cur_roi_mode == vehicle_roi_s::VEHICLE_ROI_WPNEXT) {
			_read_control_data_from_position_setpoint_sub();
			*control_data = &_control_data;
		}
	}

	return 0;
}

void InputMavlinkROI::_read_control_data_from_position_setpoint_sub()
{
	position_setpoint_triplet_s position_setpoint_triplet;
	orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
	_control_data.type_data.lonlat.lon = position_setpoint_triplet.next.lon;
	_control_data.type_data.lonlat.lat = position_setpoint_triplet.next.lat;
	_control_data.type_data.lonlat.altitude = position_setpoint_triplet.next.alt;
}

void InputMavlinkROI::print_status()
{
	PX4_INFO("Input: Mavlink (ROI)");
}

InputMavlinkCmdMount::InputMavlinkCmdMount()
{
}

InputMavlinkCmdMount::~InputMavlinkCmdMount()
{
	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
	}
}

int InputMavlinkCmdMount::initialize()
{
	if ((_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command))) < 0) {
		return -errno;
	}

	return 0;
}


int InputMavlinkCmdMount::update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	// Default to notify that there was no change.
	*control_data = nullptr;

	const int num_poll = 1;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = 		_vehicle_command_sub;
	polls[0].events = 	POLLIN;

	int ret = px4_poll(polls, num_poll, timeout_ms);

	if (ret < 0) {
		return -errno;
	}

	if (ret == 0) {
		// Timeout control_data already null.

	} else {
		if (polls[0].revents & POLLIN) {
			vehicle_command_s vehicle_command;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &vehicle_command);

			for (int i = 0; i < 3; ++i) {
				_control_data.stabilize_axis[i] = _stabilize[i];
			}

			_control_data.gimbal_shutter_retract = false;

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL) {

				switch ((int)vehicle_command.param7) {
				case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
					_control_data.gimbal_shutter_retract = true;

				//no break
				case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
					_control_data.type = ControlData::Type::Neutral;

					*control_data = &_control_data;
					break;

				case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
					_control_data.type = ControlData::Type::Angle;
					_control_data.type_data.angle.is_speed[0] = false;
					_control_data.type_data.angle.is_speed[1] = false;
					_control_data.type_data.angle.is_speed[2] = false;
					// vmount spec has roll on channel 0, MAVLink spec has pitch on channel 0
					_control_data.type_data.angle.angles[0] = vehicle_command.param2 * M_DEG_TO_RAD_F;
					// vmount spec has pitch on channel 1, MAVLink spec has roll on channel 1
					_control_data.type_data.angle.angles[1] = vehicle_command.param1 * M_DEG_TO_RAD_F;
					// both specs have yaw on channel 2
					_control_data.type_data.angle.angles[2] = vehicle_command.param3 * M_DEG_TO_RAD_F;

					*control_data = &_control_data;
					break;

				case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
					break;

				case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
					control_data_set_lon_lat(vehicle_command.param2, vehicle_command.param1, vehicle_command.param3);

					*control_data = &_control_data;
					break;
				}

				_ack_vehicle_command(vehicle_command.command);

			} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE) {
				_stabilize[0] = (uint8_t) vehicle_command.param2 == 1;
				_stabilize[1] = (uint8_t) vehicle_command.param3 == 1;
				_stabilize[2] = (uint8_t) vehicle_command.param4 == 1;
				_control_data.type = ControlData::Type::Neutral; //always switch to neutral position

				*control_data = &_control_data;
				_ack_vehicle_command(vehicle_command.command);
			}
		}

	}

	return 0;
}

void InputMavlinkCmdMount::_ack_vehicle_command(uint16_t command)
{
	vehicle_command_ack_s vehicle_command_ack;
	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = command;
	vehicle_command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

	if (_vehicle_command_ack_pub == nullptr) {
		_vehicle_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &vehicle_command_ack,
					   vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _vehicle_command_ack_pub, &vehicle_command_ack);
	}

}

void InputMavlinkCmdMount::print_status()
{
	PX4_INFO("Input: Mavlink (CMD_MOUNT)");
}


} /* namespace vmount */

