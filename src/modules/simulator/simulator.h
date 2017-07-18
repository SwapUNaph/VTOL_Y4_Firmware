/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file simulator.h
 * A device simulator
 */

#pragma once

#include <px4_posix.h>
#include <uORB/topics/hil_sensor.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <systemlib/perf_counter.h>
#include <systemlib/battery.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <v1.0/mavlink_types.h>
#include <v1.0/common/mavlink.h>
#include <geo/geo.h>
namespace simulator
{

// FIXME - what is the endianness of these on actual device?
#pragma pack(push, 1)
struct RawAccelData {
	float temperature;
	float x;
	float y;
	float z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawMagData {
	float temperature;
	float x;
	float y;
	float z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawMPUData {
	float	accel_x;
	float	accel_y;
	float	accel_z;
	float	temp;
	float	gyro_x;
	float	gyro_y;
	float	gyro_z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawBaroData {
	float pressure;
	float altitude;
	float temperature;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawAirspeedData {
	float temperature;
	float diff_pressure;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawGPSData {
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint16_t eph;
	uint16_t epv;
	uint16_t vel;
	int16_t vn;
	int16_t ve;
	int16_t vd;
	uint16_t cog;
	uint8_t fix_type;
	uint8_t satellites_visible;
};
#pragma pack(pop)

template <typename RType> class Report
{
public:
	Report(int readers) :
		_readidx(0),
		_max_readers(readers),
		_report_len(sizeof(RType))
	{
		px4_sem_init(&_lock, 0, _max_readers);
	}

	~Report() {};

	bool copyData(void *outbuf, int len)
	{
		if (len != _report_len) {
			return false;
		}

		read_lock();
		memcpy(outbuf, &_buf[_readidx], _report_len);
		read_unlock();
		return true;
	}
	void writeData(void *inbuf)
	{
		write_lock();
		memcpy(&_buf[!_readidx], inbuf, _report_len);
		_readidx = !_readidx;
		write_unlock();
	}

protected:
	void read_lock() { px4_sem_wait(&_lock); }
	void read_unlock() { px4_sem_post(&_lock); }
	void write_lock()
	{
		for (int i = 0; i < _max_readers; i++) {
			px4_sem_wait(&_lock);
		}
	}
	void write_unlock()
	{
		for (int i = 0; i < _max_readers; i++) {
			px4_sem_post(&_lock);
		}
	}

	int _readidx;
	px4_sem_t _lock;
	const int _max_readers;
	const int _report_len;
	RType _buf[2];
};

};

class Simulator
{
public:
	static Simulator *getInstance();

	enum sim_dev_t {
		SIM_GYRO,
		SIM_ACCEL,
		SIM_MAG
	};

	struct sample {
		float x;
		float y;
		float z;
		sample() : x(0), y(0), z(0) {}
		sample(float a, float b, float c) : x(a), y(b), z(c) {}
	};

	static int start(int argc, char *argv[]);

	bool getRawAccelReport(uint8_t *buf, int len);
	bool getMagReport(uint8_t *buf, int len);
	bool getMPUReport(uint8_t *buf, int len);
	bool getBaroSample(uint8_t *buf, int len);
	bool getGPSSample(uint8_t *buf, int len);
	bool getAirspeedSample(uint8_t *buf, int len);

	void write_MPU_data(void *buf);
	void write_accel_data(void *buf);
	void write_mag_data(void *buf);
	void write_baro_data(void *buf);
	void write_gps_data(void *buf);
	void write_airspeed_data(void *buf);

	bool isInitialized() { return _initialized; }

private:
	Simulator() :
		_accel(1),
		_mpu(1),
		_baro(1),
		_mag(1),
		_gps(1),
		_airspeed(1),
		_perf_accel(perf_alloc_once(PC_ELAPSED, "sim_accel_delay")),
		_perf_mpu(perf_alloc_once(PC_ELAPSED, "sim_mpu_delay")),
		_perf_baro(perf_alloc_once(PC_ELAPSED, "sim_baro_delay")),
		_perf_mag(perf_alloc_once(PC_ELAPSED, "sim_mag_delay")),
		_perf_gps(perf_alloc_once(PC_ELAPSED, "sim_gps_delay")),
		_perf_airspeed(perf_alloc_once(PC_ELAPSED, "sim_airspeed_delay")),
		_perf_sim_delay(perf_alloc_once(PC_ELAPSED, "sim_network_delay")),
		_perf_sim_interval(perf_alloc(PC_INTERVAL, "sim_network_interval")),
		_accel_pub(nullptr),
		_baro_pub(nullptr),
		_gyro_pub(nullptr),
		_mag_pub(nullptr),
		_flow_pub(nullptr),
		_dist_pub(nullptr),
		_battery_pub(nullptr),
		_initialized(false),
		_system_type(0)
#ifndef __PX4_QURT
		,
		_rc_channels_pub(nullptr),
		_attitude_pub(nullptr),
		_gpos_pub(nullptr),
		_lpos_pub(nullptr),
		_actuator_outputs_sub{},
		_vehicle_attitude_sub(-1),
		_manual_sub(-1),
		_vehicle_status_sub(-1),
		_hil_local_proj_ref(),
		_hil_local_proj_inited(false),
		_hil_ref_lat(0),
		_hil_ref_lon(0),
		_hil_ref_alt(0),
		_hil_ref_timestamp(0),
		_rc_input{},
		_actuators{},
		_attitude{},
		_manual{},
		_vehicle_status{}
#endif
	{
		// We need to know the type for the correct mapping from
		// actuator controls to the hil actuator message.
		param_t param_system_type = param_find("MAV_TYPE");
		param_get(param_system_type, &_system_type);

		for (unsigned i = 0; i < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); i++)
		{
			_actuator_outputs_sub[i] = -1;
		}
	}
	~Simulator()
	{
		if (_instance != nullptr) {
			delete _instance;
		}

		_instance = NULL;
	}

	void initializeSensorData();

	static Simulator *_instance;

	// simulated sensor instances
	simulator::Report<simulator::RawAccelData>	_accel;
	simulator::Report<simulator::RawMPUData>	_mpu;
	simulator::Report<simulator::RawBaroData>	_baro;
	simulator::Report<simulator::RawMagData>	_mag;
	simulator::Report<simulator::RawGPSData>	_gps;
	simulator::Report<simulator::RawAirspeedData> _airspeed;

	perf_counter_t _perf_accel;
	perf_counter_t _perf_mpu;
	perf_counter_t _perf_baro;
	perf_counter_t _perf_mag;
	perf_counter_t _perf_gps;
	perf_counter_t _perf_airspeed;
	perf_counter_t _perf_sim_delay;
	perf_counter_t _perf_sim_interval;

	// uORB publisher handlers
	orb_advert_t _accel_pub;
	orb_advert_t _baro_pub;
	orb_advert_t _gyro_pub;
	orb_advert_t _mag_pub;
	orb_advert_t _flow_pub;
	orb_advert_t _dist_pub;
	orb_advert_t _battery_pub;

	bool _initialized;

	// Lib used to do the battery calculations.
	Battery _battery;

	// For param MAV_TYPE
	int32_t _system_type;

	// class methods
	int publish_sensor_topics(mavlink_hil_sensor_t *imu);
	int publish_flow_topic(mavlink_hil_optical_flow_t *flow);
	int publish_distance_topic(mavlink_distance_sensor_t *dist);

#ifndef __PX4_QURT
	// uORB publisher handlers
	orb_advert_t _rc_channels_pub;
	orb_advert_t _attitude_pub;
	orb_advert_t _gpos_pub;
	orb_advert_t _lpos_pub;

	// uORB subscription handlers
	int _actuator_outputs_sub[ORB_MULTI_MAX_INSTANCES];
	int _vehicle_attitude_sub;
	int _manual_sub;
	int _vehicle_status_sub;

	// hil map_ref data
	struct map_projection_reference_s _hil_local_proj_ref;
	bool _hil_local_proj_inited;
	double _hil_ref_lat;
	double _hil_ref_lon;
	float _hil_ref_alt;
	uint64_t _hil_ref_timestamp;

	// uORB data containers
	struct rc_input_values _rc_input;
	struct actuator_outputs_s _actuators[ORB_MULTI_MAX_INSTANCES];
	struct vehicle_attitude_s _attitude;
	struct manual_control_setpoint_s _manual;
	struct vehicle_status_s _vehicle_status;

	void poll_topics();
	void handle_message(mavlink_message_t *msg, bool publish);
	void send_controls();
	void pollForMAVLinkMessages(bool publish, int udp_port);

	void pack_actuator_message(mavlink_hil_actuator_controls_t &actuator_msg, unsigned index);
	void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);
	void update_sensors(mavlink_hil_sensor_t *imu);
	void update_gps(mavlink_hil_gps_t *gps_sim);
	static void *sending_trampoline(void *);
	void send();
#endif
};
