/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file px4flow.cpp
 * @author Dominik Honegger
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PX4FLOW module connected via I2C.
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <conversion/rotation.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_px4flow.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define I2C_FLOW_ADDRESS 		0x42	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49

/* PX4FLOW Registers addresses */
#define PX4FLOW_REG			0x16	///< Measure Register 22

#define PX4FLOW_CONVERSION_INTERVAL	100000	///< in microseconds! 20000 = 50 Hz 100000 = 10Hz
#define PX4FLOW_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed

#define PX4FLOW_MAX_DISTANCE 5.0f
#define PX4FLOW_MIN_DISTANCE 0.3f

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#include "i2c_frame.h"

struct i2c_frame f;
struct i2c_integral_frame f_integral;

class PX4FLOW: public device::I2C
{
public:
	PX4FLOW(int bus, int address = I2C_FLOW_ADDRESS, enum Rotation rotation = (enum Rotation)0);
	virtual ~PX4FLOW();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

protected:
	virtual int			probe();

private:

	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	int				_measure_ticks;
	bool				_collect_phase;
	int			_class_instance;
	int			_orb_class_instance;
	orb_advert_t		_px4flow_topic;
	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	enum Rotation				_sensor_rotation;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int					probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void				start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void				stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void				cycle();
	int					measure();
	int					collect();
	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void			cycle_trampoline(void *arg);

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int px4flow_main(int argc, char *argv[]);

PX4FLOW::PX4FLOW(int bus, int address, enum Rotation rotation) :
	I2C("PX4FLOW", PX4FLOW0_DEVICE_PATH, bus, address, PX4FLOW_I2C_MAX_BUS_SPEED), /* 100-400 KHz */
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_px4flow_topic(nullptr),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "px4f_read")),
	_comms_errors(perf_alloc(PC_COUNT, "px4f_com_err")),
	_sensor_rotation(rotation)
{
	// disable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PX4FLOW::~PX4FLOW()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PX4FLOW::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(optical_flow_s));

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
		}

	} else {
		DEVICE_LOG("not primary range device, not advertising");
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	/* get yaw rotation from sensor frame to body frame */
	param_t rot = param_find("SENS_FLOW_ROT");

	/* only set it if the parameter exists */
	if (rot != PARAM_INVALID) {
		int32_t val = 6; // the recommended installation for the flow sensor is with the Y sensor axis forward
		param_get(rot, &val);

		_sensor_rotation = (enum Rotation)val;
	}

	return ret;
}

int
PX4FLOW::probe()
{
	uint8_t val[I2C_FRAME_SIZE];

	// to be sure this is not a ll40ls Lidar (which can also be on
	// 0x42) we check if a I2C_FRAME_SIZE byte transfer works from address
	// 0. The ll40ls gives an error for that, whereas the flow
	// happily returns some data
	if (transfer(nullptr, 0, &val[0], 22) != OK) {
		return -EIO;
	}

	// that worked, so start a measurement cycle
	return measure();
}

int
PX4FLOW::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(PX4FLOW_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(PX4FLOW_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCSROTATION:
		_sensor_rotation = (enum Rotation)arg;
		return OK;

	case SENSORIOCGROTATION:
		return _sensor_rotation;

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
PX4FLOW::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct optical_flow_s);
	struct optical_flow_s *rbuf = reinterpret_cast<struct optical_flow_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
PX4FLOW::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = PX4FLOW_REG;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
PX4FLOW::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE] = { 0 };

	perf_begin(_sample_perf);

	if (PX4FLOW_REG == 0x00) {
		ret = transfer(nullptr, 0, &val[0], I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE);
	}

	if (PX4FLOW_REG == 0x16) {
		ret = transfer(nullptr, 0, &val[0], I2C_INTEGRAL_FRAME_SIZE);
	}

	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	if (PX4FLOW_REG == 0) {
		memcpy(&f, val, I2C_FRAME_SIZE);
		memcpy(&f_integral, &(val[I2C_FRAME_SIZE]), I2C_INTEGRAL_FRAME_SIZE);
	}

	if (PX4FLOW_REG == 0x16) {
		memcpy(&f_integral, val, I2C_INTEGRAL_FRAME_SIZE);
	}


	struct optical_flow_s report;

	report.timestamp = hrt_absolute_time();

	report.pixel_flow_x_integral = static_cast<float>(f_integral.pixel_flow_x_integral) / 10000.0f;//convert to radians

	report.pixel_flow_y_integral = static_cast<float>(f_integral.pixel_flow_y_integral) / 10000.0f;//convert to radians

	report.frame_count_since_last_readout = f_integral.frame_count_since_last_readout;

	report.ground_distance_m = static_cast<float>(f_integral.ground_distance) / 1000.0f;//convert to meters

	report.quality = f_integral.qual; //0:bad ; 255 max quality

	report.gyro_x_rate_integral = static_cast<float>(f_integral.gyro_x_rate_integral) / 10000.0f; //convert to radians

	report.gyro_y_rate_integral = static_cast<float>(f_integral.gyro_y_rate_integral) / 10000.0f; //convert to radians

	report.gyro_z_rate_integral = static_cast<float>(f_integral.gyro_z_rate_integral) / 10000.0f; //convert to radians

	report.integration_timespan = f_integral.integration_timespan; //microseconds

	report.time_since_last_sonar_update = f_integral.sonar_timestamp;//microseconds

	report.gyro_temperature = f_integral.gyro_temperature;//Temperature * 100 in centi-degrees Celsius

	report.sensor_id = 0;

	/* rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT */
	float zeroval = 0.0f;

	rotate_3f(_sensor_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval);

	rotate_3f(_sensor_rotation, report.gyro_x_rate_integral, report.gyro_y_rate_integral, report.gyro_z_rate_integral);

	if (_px4flow_topic == nullptr) {
		_px4flow_topic = orb_advertise(ORB_ID(optical_flow), &report);

	} else {
		/* publish it */
		orb_publish(ORB_ID(optical_flow), _px4flow_topic, &report);
	}

	/* publish to the distance_sensor topic as well */
	struct distance_sensor_s distance_report;
	distance_report.timestamp = report.timestamp;
	distance_report.min_distance = PX4FLOW_MIN_DISTANCE;
	distance_report.max_distance = PX4FLOW_MAX_DISTANCE;
	distance_report.current_distance = report.ground_distance_m;
	distance_report.covariance = 0.0f;
	distance_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	/* TODO: the ID needs to be properly set */
	distance_report.id = 0;
	distance_report.orientation = 8;

	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &distance_report);

	/* post a report to the ring */
	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
PX4FLOW::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PX4FLOW::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_OPTICALFLOW;

	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
PX4FLOW::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PX4FLOW::cycle_trampoline(void *arg)
{
	PX4FLOW *dev = (PX4FLOW *)arg;

	dev->cycle();
}

void
PX4FLOW::cycle()
{
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* perform collection */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}

	work_queue(HPWORK, &_work, (worker_t)&PX4FLOW::cycle_trampoline, this,
		   _measure_ticks);

}

void
PX4FLOW::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace px4flow
{

PX4FLOW	*g_dev = nullptr;
bool start_in_progress = false;

const int START_RETRY_COUNT = 5;
const int START_RETRY_TIMEOUT = 1000;

int	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
int
start()
{
	int fd;

	/* entry check: */
	if (start_in_progress) {
		warnx("start already in progress");
		return 1;
	}

	start_in_progress = true;

	if (g_dev != nullptr) {
		start_in_progress = false;
		warnx("already started");
		return 1;
	}

	warnx("scanning I2C buses for device..");

	int retry_nr = 0;

	while (1) {
		const int busses_to_try[] = {
			PX4_I2C_BUS_EXPANSION,
#ifdef PX4_I2C_BUS_ESC
			PX4_I2C_BUS_ESC,
#endif
#ifdef PX4_I2C_BUS_ONBOARD
			PX4_I2C_BUS_ONBOARD,
#endif
			-1
		};

		const int *cur_bus = busses_to_try;

		while (*cur_bus != -1) {
			/* create the driver */
			/* warnx("trying bus %d", *cur_bus); */
			g_dev = new PX4FLOW(*cur_bus);

			if (g_dev == nullptr) {
				/* this is a fatal error */
				break;
			}

			/* init the driver: */
			if (OK == g_dev->init()) {
				/* success! */
				break;
			}

			/* destroy it again because it failed. */
			delete g_dev;
			g_dev = nullptr;

			/* try next! */
			cur_bus++;
		}

		/* check whether we found it: */
		if (*cur_bus != -1) {

			/* check for failure: */
			if (g_dev == nullptr) {
				break;
			}

			/* set the poll rate to default, starts automatic data collection */
			fd = open(PX4FLOW0_DEVICE_PATH, O_RDONLY);

			if (fd < 0) {
				break;
			}

			if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX) < 0) {
				break;
			}

			/* success! */
			start_in_progress = false;
			return 0;
		}

		if (retry_nr < START_RETRY_COUNT) {
			/* lets not be too verbose */
			// warnx("PX4FLOW not found on I2C busses. Retrying in %d ms. Giving up in %d retries.", START_RETRY_TIMEOUT, START_RETRY_COUNT - retry_nr);
			usleep(START_RETRY_TIMEOUT * 1000);
			retry_nr++;

		} else {
			break;
		}
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	start_in_progress = false;
	return 1;
}

/**
 * Stop the driver
 */
void
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct optical_flow_s report;
	ssize_t sz;
	int ret;

	int fd = open(PX4FLOW0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'px4flow start' if the driver is not running", PX4FLOW0_DEVICE_PATH);
	}


	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		warnx("immediate read failed");
	}

	warnx("single read");
	warnx("pixel_flow_x_integral: %i", f_integral.pixel_flow_x_integral);
	warnx("pixel_flow_y_integral: %i", f_integral.pixel_flow_y_integral);
	warnx("framecount_integral: %u",
	      f_integral.frame_count_since_last_readout);

	/* start the sensor polling at 10Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 10)) {
		errx(1, "failed to set 10Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);

		warnx("framecount_total: %u", f.frame_count);
		warnx("framecount_integral: %u",
		      f_integral.frame_count_since_last_readout);
		warnx("pixel_flow_x_integral: %i", f_integral.pixel_flow_x_integral);
		warnx("pixel_flow_y_integral: %i", f_integral.pixel_flow_y_integral);
		warnx("gyro_x_rate_integral: %i", f_integral.gyro_x_rate_integral);
		warnx("gyro_y_rate_integral: %i", f_integral.gyro_y_rate_integral);
		warnx("gyro_z_rate_integral: %i", f_integral.gyro_z_rate_integral);
		warnx("integration_timespan [us]: %u", f_integral.integration_timespan);
		warnx("ground_distance: %0.2f m",
		      (double) f_integral.ground_distance / 1000);
		warnx("time since last sonar update [us]: %i",
		      f_integral.sonar_timestamp);
		warnx("quality integration average : %i", f_integral.qual);
		warnx("quality : %i", f.qual);


	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(PX4FLOW0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
px4flow_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		return px4flow::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		px4flow::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		px4flow::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		px4flow::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		px4flow::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
