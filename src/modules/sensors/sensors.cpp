/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * PX4 Flight Core transitional mapping layer.
 *
 * This app / class mapps the PX4 middleware layer / drivers to the application
 * layer of the PX4 Flight Core. Individual sensors can be accessed directly as
 * well instead of relying on the sensor_combined topic.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#include <board_config.h>

#include <px4_adc.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_px4flow.h>

#include <systemlib/airspeed.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/battery.h>

#include <conversion/rotation.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/sensor_preflight.h>

#include <DevMgr.hpp>

#include "parameters.h"
#include "rc_update.h"
#include "voted_sensors_update.h"

using namespace DriverFramework;
using namespace sensors;

/**
 * Analog layout:
 * FMU:
 * IN2 - battery voltage
 * IN3 - battery current
 * IN4 - 5V sense
 * IN10 - spare (we could actually trim these from the set)
 * IN11 - spare on FMUv2 & v3, RC RSSI on FMUv4
 * IN12 - spare (we could actually trim these from the set)
 * IN13 - aux1 on FMUv2, unavaible on v3 & v4
 * IN14 - aux2 on FMUv2, unavaible on v3 & v4
 * IN15 - pressure sensor on FMUv2, unavaible on v3 & v4
 *
 * IO:
 * IN4 - servo supply rail
 * IN5 - analog RSSI on FMUv2 & v3
 *
 * The channel definitions (e.g., ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL, and ADC_AIRSPEED_VOLTAGE_CHANNEL) are defined in board_config.h
 */


/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f
#define STICK_ON_OFF_LIMIT		0.75f

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sensors_main(int argc, char *argv[]);

class Sensors
{
public:
	/**
	 * Constructor
	 */
	Sensors(bool hil_enabled);

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Sensors();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();


	void	print_status();

private:
	DevHandle 	_h_adc;				/**< ADC driver handle */
	hrt_abstime	_last_adc;			/**< last time we took input from the ADC */

	volatile bool 	_task_should_exit;		/**< if true, sensor task should exit */
	int 		_sensors_task;			/**< task handle for sensor task */

	const bool	_hil_enabled;			/**< if true, HIL is active */
	bool		_armed;				/**< arming status of the vehicle */

	int		_actuator_ctrl_0_sub;		/**< attitude controls sub */
	int		_diff_pres_sub;			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub;		/**< vehicle control mode subscription */
	int 		_params_sub;			/**< notification of parameter updates */

	orb_advert_t	_sensor_pub;			/**< combined sensor data topic */
	orb_advert_t	_battery_pub;			/**< battery status */
	orb_advert_t	_airspeed_pub;			/**< airspeed */
	orb_advert_t	_diff_pres_pub;			/**< differential_pressure */
	orb_advert_t	_sensor_preflight;		/**< sensor preflight topic */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

	struct battery_status_s _battery_status;	/**< battery status */
	struct differential_pressure_s _diff_pres;
	struct airspeed_s _airspeed;

	Battery		_battery;			/**< Helper lib to publish battery_status topic. */

	Parameters		_parameters;			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles;		/**< handles for interesting parameters */

	RCUpdate		_rc_update;
	VotedSensorsUpdate _voted_sensors_update;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Do adc-related initialisation.
	 */
	int		adc_init();

	/**
	 * Poll the differential pressure sensor for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		diff_pres_poll(struct sensor_combined_s &raw);

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll(struct sensor_combined_s &raw);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace sensors
{

Sensors	*g_sensors = nullptr;
}

Sensors::Sensors(bool hil_enabled) :
	_h_adc(),
	_last_adc(0),

	_task_should_exit(true),
	_sensors_task(-1),
	_hil_enabled(hil_enabled),
	_armed(false),

	_actuator_ctrl_0_sub(-1),
	_diff_pres_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),

	/* publications */
	_sensor_pub(nullptr),
	_battery_pub(nullptr),
	_airspeed_pub(nullptr),
	_diff_pres_pub(nullptr),
	_sensor_preflight(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
	_airspeed_validator(),

	_rc_update(_parameters),
	_voted_sensors_update(_parameters, hil_enabled)
{
	memset(&_diff_pres, 0, sizeof(_diff_pres));
	memset(&_parameters, 0, sizeof(_parameters));

	initialize_parameter_handles(_parameter_handles);

	_airspeed_validator.set_timeout(300000);
	_airspeed_validator.set_equal_value_threshold(100);
}

Sensors::~Sensors()
{
	if (_sensors_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_sensors_task);
				break;
			}
		} while (_sensors_task != -1);
	}

	sensors::g_sensors = nullptr;
}

int
Sensors::parameters_update()
{
	/* read the parameter values into _parameters */
	int ret = update_parameters(_parameter_handles, _parameters);

	if (ret) {
		return ret;
	}

	_rc_update.update_rc_functions();
	_voted_sensors_update.parameters_update();

	/* update barometer qnh setting */
	DevHandle h_baro;
	DevMgr::getHandle(BARO0_DEVICE_PATH, h_baro);

#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	// TODO: this needs fixing for QURT and Raspberry Pi
	if (!h_baro.isValid()) {
		if (!_hil_enabled) { // in HIL we don't have a baro
			PX4_ERR("no barometer found on %s (%d)", BARO0_DEVICE_PATH, h_baro.getError());
			ret = PX4_ERROR;
		}

	} else {
		int baroret = h_baro.ioctl(BAROIOCSMSLPRESSURE, (unsigned long)(_parameters.baro_qnh * 100));

		if (baroret) {
			PX4_ERR("qnh for baro could not be set");
			ret = PX4_ERROR;
		}
	}

#endif

	return ret;
}


int
Sensors::adc_init()
{

	DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	if (!_h_adc.isValid()) {
		PX4_ERR("no ADC found: %s (%d)", ADC0_DEVICE_PATH, _h_adc.getError());
		return PX4_ERROR;
	}

	return OK;
}

void
Sensors::diff_pres_poll(struct sensor_combined_s &raw)
{
	bool updated;
	orb_check(_diff_pres_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(differential_pressure), _diff_pres_sub, &_diff_pres);

		float air_temperature_celsius = (_diff_pres.temperature > -300.0f) ? _diff_pres.temperature :
						(raw.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

		_airspeed.timestamp = _diff_pres.timestamp;

		/* push data into validator */
		_airspeed_validator.put(_airspeed.timestamp, _diff_pres.differential_pressure_raw_pa, _diff_pres.error_count,
					ORB_PRIO_HIGH);

		_airspeed.confidence = _airspeed_validator.confidence(hrt_absolute_time());

		/* don't risk to feed negative airspeed into the system */
		_airspeed.indicated_airspeed_m_s = math::max(0.0f,
						   calc_indicated_airspeed(_diff_pres.differential_pressure_filtered_pa));

		_airspeed.true_airspeed_m_s = math::max(0.0f,
							calc_true_airspeed(_diff_pres.differential_pressure_filtered_pa + _voted_sensors_update.baro_pressure(),
									_voted_sensors_update.baro_pressure(), air_temperature_celsius));

		_airspeed.true_airspeed_unfiltered_m_s = math::max(0.0f,
				calc_true_airspeed(_diff_pres.differential_pressure_raw_pa + _voted_sensors_update.baro_pressure(),
						   _voted_sensors_update.baro_pressure(), air_temperature_celsius));

		_airspeed.air_temperature_celsius = air_temperature_celsius;
		_airspeed.differential_pressure_filtered_pa = _diff_pres.differential_pressure_filtered_pa;

		int instance;
		orb_publish_auto(ORB_ID(airspeed), &_airspeed_pub, &_airspeed, &instance, ORB_PRIO_DEFAULT);
	}
}

void
Sensors::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
	}
}

void
Sensors::parameter_update_poll(bool forced)
{
	bool param_updated = false;

	/* Check if any parameter has changed */
	orb_check(_params_sub, &param_updated);

	if (param_updated || forced) {
		/* read from param to clear updated flag */
		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &update);

		parameters_update();

		/* update airspeed scale */
		int fd = px4_open(AIRSPEED0_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */
		if (fd >= 0) {
			struct airspeed_scale airscale = {
				_parameters.diff_pres_offset_pa,
				1.0f,
			};

			if (OK != px4_ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				warn("WARNING: failed to set scale / offsets for airspeed sensor");
			}

			px4_close(fd);
		}

		_battery.updateParams();
	}
}

void
Sensors::adc_poll(struct sensor_combined_s &raw)
{
	/* only read if not in HIL mode */
	if (_hil_enabled) {
		return;
	}

	hrt_abstime t = hrt_absolute_time();

	/* rate limit to 100 Hz */
	if (t - _last_adc >= 10000) {
		/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
		struct adc_msg_s buf_adc[12];
		/* read all channels available */
		int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));

		float bat_voltage_v = 0.0f;
		float bat_current_a = 0.0f;
		bool updated_battery = false;

		if (ret >= (int)sizeof(buf_adc[0])) {

			/* Read add channels we got */
			for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {

				/* look for specific channels and process the raw voltage to measurement data */
				if (ADC_BATTERY_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {
					/* Voltage in volts */
					bat_voltage_v = (buf_adc[i].am_data * _parameters.battery_voltage_scaling) * _parameters.battery_v_div;

					if (bat_voltage_v > 0.5f) {
						updated_battery = true;
					}

				} else if (ADC_BATTERY_CURRENT_CHANNEL == buf_adc[i].am_channel) {
					bat_current_a = ((buf_adc[i].am_data * _parameters.battery_current_scaling)
							 - _parameters.battery_current_offset) * _parameters.battery_a_per_v;

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

				} else if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

					/* calculate airspeed, raw is the difference from */
					float voltage = (float)(buf_adc[i].am_data) * 3.3f / 4096.0f * 2.0f;  // V_ref/4096 * (voltage divider factor)

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 */
					if (voltage > 0.4f && (_parameters.diff_pres_analog_scale > 0.0f)) {

						float diff_pres_pa_raw = voltage * _parameters.diff_pres_analog_scale - _parameters.diff_pres_offset_pa;

						_diff_pres.timestamp = t;
						_diff_pres.differential_pressure_raw_pa = diff_pres_pa_raw;
						_diff_pres.differential_pressure_filtered_pa = (_diff_pres.differential_pressure_filtered_pa * 0.9f) +
								(diff_pres_pa_raw * 0.1f);
						_diff_pres.temperature = -1000.0f;

						int instance;
						orb_publish_auto(ORB_ID(differential_pressure), &_diff_pres_pub, &_diff_pres, &instance,
								 ORB_PRIO_DEFAULT);
					}

#endif
				}
			}

			if (_parameters.battery_source == 0 && updated_battery) {
				actuator_controls_s ctrl;
				orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);
				_battery.updateBatteryStatus(t, bat_voltage_v, bat_current_a, ctrl.control[actuator_controls_s::INDEX_THROTTLE],
							     _armed, &_battery_status);

				int instance;
				orb_publish_auto(ORB_ID(battery_status), &_battery_pub, &_battery_status, &instance, ORB_PRIO_DEFAULT);
			}

			_last_adc = t;

		}
	}
}

void
Sensors::task_main_trampoline(int argc, char *argv[])
{
	sensors::g_sensors->task_main();
}

void
Sensors::task_main()
{

	int ret = 0;

	if (!_hil_enabled) {
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_BEBOP)
		adc_init();
#endif
	}

	struct sensor_combined_s raw = {};

	struct sensor_preflight_s preflt = {};

	_rc_update.init();

	_voted_sensors_update.init(raw);

	/* (re)load params and calibration */
	parameter_update_poll(true);

	/*
	 * do subscriptions
	 */
	_diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));

	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	_actuator_ctrl_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));

	_battery.reset(&_battery_status);

	/* get a set of initial values */
	_voted_sensors_update.sensors_poll(raw);

	diff_pres_poll(raw);

	_rc_update.rc_parameter_map_poll(_parameter_handles, true /* forced */);

	/* advertise the sensor_combined topic and make the initial publication */
	_sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);

	/* advertise the sensor_preflight topic and make the initial publication */
	preflt.accel_inconsistency_m_s_s = 0.0f;

	preflt.gyro_inconsistency_rad_s = 0.0f;

	_sensor_preflight = orb_advertise(ORB_ID(sensor_preflight), &preflt);

	/* wakeup source */
	px4_pollfd_struct_t poll_fds = {};

	poll_fds.events = POLLIN;

	_task_should_exit = false;

	uint64_t last_config_update = hrt_absolute_time();

	while (!_task_should_exit) {

		/* use the best-voted gyro to pace output */
		poll_fds.fd = _voted_sensors_update.best_gyro_fd();

		/* wait for up to 50ms for data (Note that this implies, we can have a fail-over time of 50ms,
		 * if a gyro fails) */
		int pret = px4_poll(&poll_fds, 1, 50);

		/* if pret == 0 it timed out - periodic check for _task_should_exit, etc. */

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			/* if the polling operation failed because no gyro sensor is available yet,
			 * then attempt to subscribe once again
			 */
			if (_voted_sensors_update.num_gyros() == 0) {
				_voted_sensors_update.initialize_sensors();
			}

			usleep(1000);

			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* the timestamp of the raw struct is updated by the gyro_poll() method (this makes the gyro
		 * a mandatory sensor) */
		_voted_sensors_update.sensors_poll(raw);

		/* check battery voltage */
		adc_poll(raw);

		diff_pres_poll(raw);

		if (raw.timestamp > 0) {

			_voted_sensors_update.set_relative_timestamps(raw);

			orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);

			_voted_sensors_update.check_failover();

			/* If the the vehicle is disarmed calculate the length of the maximum difference between
			 * IMU units as a consistency metric and publish to the sensor preflight topic
			*/
			if (!_armed) {
				_voted_sensors_update.calc_accel_inconsistency(preflt);
				_voted_sensors_update.calc_gyro_inconsistency(preflt);
				orb_publish(ORB_ID(sensor_preflight), _sensor_preflight, &preflt);

			}

			//_voted_sensors_update.check_vibration(); //disabled for now, as it does not seem to be reliable
		}

		/* keep adding sensors as long as we are not armed,
		 * when not adding sensors poll for param updates
		 */
		if (!_armed && hrt_elapsed_time(&last_config_update) > 500 * 1000) {
			_voted_sensors_update.initialize_sensors();
			last_config_update = hrt_absolute_time();

		} else {

			/* check parameters for updates */
			parameter_update_poll();

			/* check rc parameter map for updates */
			_rc_update.rc_parameter_map_poll(_parameter_handles);
		}

		/* Look for new r/c input data */
		_rc_update.rc_poll(_parameter_handles);

		perf_end(_loop_perf);
	}

	orb_unsubscribe(_diff_pres_sub);
	orb_unsubscribe(_vcontrol_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_actuator_ctrl_0_sub);
	orb_unadvertise(_sensor_pub);

	_rc_update.deinit();
	_voted_sensors_update.deinit();

	_sensors_task = -1;
	px4_task_exit(ret);
}

int
Sensors::start()
{
	ASSERT(_sensors_task == -1);

	/* start the task */
	_sensors_task = px4_task_spawn_cmd("sensors",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&Sensors::task_main_trampoline,
					   nullptr);

	/* wait until the task is up and running or has failed */
	while (_sensors_task > 0 && _task_should_exit) {
		usleep(100);
	}

	if (_sensors_task < 0) {
		return -PX4_ERROR;
	}

	return OK;
}

void Sensors::print_status()
{
	_voted_sensors_update.print_status();
}


int sensors_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: sensors {start|stop|status}");
		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (sensors::g_sensors != nullptr) {
			PX4_INFO("already running");
			return 0;
		}

		bool hil_enabled = false;

		if (argc > 2 && !strcmp(argv[2], "-hil")) {
			hil_enabled = true;
		}

		sensors::g_sensors = new Sensors(hil_enabled);

		if (sensors::g_sensors == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		if (OK != sensors::g_sensors->start()) {
			delete sensors::g_sensors;
			sensors::g_sensors = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (sensors::g_sensors == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		delete sensors::g_sensors;
		sensors::g_sensors = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (sensors::g_sensors) {
			sensors::g_sensors->print_status();
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	PX4_ERR("unrecognized command");
	return 1;
}
