/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * Driver for the Invensense mpu9250 connected via I2C or SPI.
 *
 * @authors Andrew Tridgell
 *          Robert Dickenson
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mpu9250.h"

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_MAG		"/dev/mpu9250_mag"
#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu9250_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu9250_gyro_ext"
#define MPU_DEVICE_PATH_MAG_EXT 	"/dev/mpu9250_mag_ext"

/** driver 'main' command */
extern "C" { __EXPORT int mpu9250_main(int argc, char *argv[]); }


enum MPU9250_BUS {
	MPU9250_BUS_ALL = 0,
	MPU9250_BUS_I2C_INTERNAL,
	MPU9250_BUS_I2C_EXTERNAL,
	MPU9250_BUS_SPI_INTERNAL,
	MPU9250_BUS_SPI_EXTERNAL
};


/**
 * Local functions in support of the shell command.
 */
namespace mpu9250
{

/*
  list of supported bus configurations
 */

struct mpu9250_bus_option {
	enum MPU9250_BUS busid;
	const char *accelpath;
	const char *gyropath;
	const char *magpath;
	MPU9250_constructor interface_constructor;
	bool magpassthrough;
	uint8_t busnum;
	MPU9250	*dev;
} bus_options[] = {
#if defined (USE_I2C)
#  if defined(PX4_I2C_BUS_ONBOARD)
	{ MPU9250_BUS_I2C_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG,  &MPU9250_I2C_interface, false, PX4_I2C_BUS_ONBOARD, NULL },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, NULL },
#  endif
#endif
#ifdef PX4_SPIDEV_MPU
	{ MPU9250_BUS_SPI_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, NULL },
#endif
#if defined(PX4_SPI_BUS_EXT)
	{ MPU9250_BUS_SPI_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_SPI_interface, true, PX4_SPI_BUS_EXT, NULL },
#endif
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))


void	start(enum MPU9250_BUS busid, enum Rotation rotation, bool external_bus);
bool	start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external_bus);
struct mpu9250_bus_option &find_bus(enum MPU9250_BUS busid);
void	stop(enum MPU9250_BUS busid);
void	test(enum MPU9250_BUS busid);
void	reset(enum MPU9250_BUS busid);
void	info(enum MPU9250_BUS busid);
void	regdump(enum MPU9250_BUS busid);
void	testerror(enum MPU9250_BUS busid);
void	usage();

/**
 * find a bus structure for a busid
 */
struct mpu9250_bus_option &find_bus(enum MPU9250_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MPU9250_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external)
{
	int fd = -1;

	if (bus.dev != nullptr) {
		warnx("%s SPI not available", external ? "External" : "Internal");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum, external);

	if (interface == nullptr) {
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	device::Device *mag_interface = nullptr;

#ifdef USE_I2C
	/* For i2c interfaces, connect to the magnetomer directly */
	bool is_i2c = bus.busid == MPU9250_BUS_I2C_INTERNAL || bus.busid == MPU9250_BUS_I2C_EXTERNAL;

	if (is_i2c) {
		mag_interface = AK8963_I2C_interface(bus.busnum, external);
	}

#endif

	bus.dev = new MPU9250(interface, mag_interface, bus.accelpath, bus.gyropath, bus.magpath, rotation);

	if (bus.dev == nullptr) {
		delete interface;
		return false;
	}

	if (OK != bus.dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(bus.accelpath, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}


	close(fd);

	return true;

fail:

	if (fd >= 0) {
		close(fd);
	}

	if (bus.dev != nullptr) {
		delete (bus.dev);
		bus.dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(enum MPU9250_BUS busid, enum Rotation rotation, bool external)
{

	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MPU9250_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MPU9250_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation, external);
	}

	exit(started ? 0 : 1);

}

void
stop(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev != nullptr) {
		delete bus.dev;
		bus.dev = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);
	accel_report a_report;
	gyro_report g_report;
	mag_report m_report;
	ssize_t sz;

	/* get the driver */
	int fd = open(bus.accelpath, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'm start')", bus.accelpath);
	}

	/* get the driver */
	int fd_gyro = open(bus.gyropath, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", bus.gyropath);
	}

	/* get the driver */
	int fd_mag = open(bus.magpath, O_RDONLY);

	if (fd_mag < 0) {
		err(1, "%s open failed", bus.magpath);
	}

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	warnx("single read");
	warnx("time:     %lld", a_report.timestamp);
	warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
	warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
	warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
	warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
	warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
	warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
	warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
	      (double)(a_report.range_m_s2 / MPU9250_ONE_G));

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
	warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);

	/* do a simple demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(m_report));
		err(1, "immediate mag read failed");
	}

	warnx("mag x: \t% 9.5f\trad/s", (double)m_report.x);
	warnx("mag y: \t% 9.5f\trad/s", (double)m_report.y);
	warnx("mag z: \t% 9.5f\trad/s", (double)m_report.z);
	warnx("mag x: \t%d\traw", (int)m_report.x_raw);
	warnx("mag y: \t%d\traw", (int)m_report.y_raw);
	warnx("mag z: \t%d\traw", (int)m_report.z_raw);
	warnx("mag range: %8.4f Ga", (double)m_report.range_ga);
	warnx("mag temp:  %8.4f\tdeg celsius", (double)m_report.temperature);

	/* reset to default polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd);
	close(fd_gyro);
	close(fd_mag);

	/* XXX add poll-rate tests here too */

	reset(busid);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);
	int fd = open(bus.accelpath, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", bus.dev);
	bus.dev->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", bus.dev);
	bus.dev->print_registers();

	exit(0);
}

/**
 * deliberately produce an error to test recovery
 */
void
testerror(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	bus.dev->test_error();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
mpu9250_main(int argc, char *argv[])
{
	enum MPU9250_BUS busid = MPU9250_BUS_ALL;
	int ch;
	bool external = false;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XISsR:")) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPU9250_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPU9250_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MPU9250_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = MPU9250_BUS_SPI_INTERNAL;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			mpu9250::usage();
			exit(0);
		}
	}

	external = (busid == MPU9250_BUS_I2C_EXTERNAL || busid == MPU9250_BUS_SPI_EXTERNAL);

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		mpu9250::start(busid, rotation, external);
	}

	if (!strcmp(verb, "stop")) {
		mpu9250::stop(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		mpu9250::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpu9250::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		mpu9250::info(busid);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		mpu9250::regdump(busid);
	}

	if (!strcmp(verb, "testerror")) {
		mpu9250::testerror(busid);
	}

	mpu9250::usage();
	exit(1);
}
