/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file config.c
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 *
 * config tool. Takes the device name as the first parameter.
 */

#include <px4_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <arch/board/board.h>

#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"

__EXPORT int config_main(int argc, char *argv[]);

static void	do_gyro(int argc, char *argv[]);
static void	do_accel(int argc, char *argv[]);
static void	do_mag(int argc, char *argv[]);
static void	do_device(int argc, char *argv[]);

int
config_main(int argc, char *argv[])
{
	bool is_device_cmd = argc >= 3 && (!strcmp(argv[2], "block") || !strcmp(argv[2], "unblock"));

	if (argc >= 2) {
		if (!is_device_cmd && !strncmp(argv[1], "/dev/gyro", 9)) {
			do_gyro(argc - 1, argv + 1);

		} else if (!is_device_cmd && !strncmp(argv[1], "/dev/accel", 10)) {
			do_accel(argc - 1, argv + 1);

		} else if (!is_device_cmd && !strncmp(argv[1], "/dev/mag", 8)) {
			do_mag(argc - 1, argv + 1);

		} else {
			do_device(argc - 1, argv + 1);
		}
	}

	errx(1, "expected a device, try '/dev/gyro', '/dev/accel', '/dev/mag'");
}

static void
do_device(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "no device path provided and command provided.");
	}

	int	fd;

	fd = open(argv[0], 0);

	if (fd < 0) {
		warn("%s", argv[0]);
		errx(1, "FATAL: no device found");

	} else {

		int	ret;

		if (argc == 2 && !strcmp(argv[1], "block")) {

			/* disable the device publications */
			ret = ioctl(fd, DEVIOCSPUBBLOCK, 1);

			if (ret) {
				errx(ret, "uORB publications could not be blocked");
			}

		} else if (argc == 2 && !strcmp(argv[1], "unblock")) {

			/* enable the device publications */
			ret = ioctl(fd, DEVIOCSPUBBLOCK, 0);

			if (ret) {
				errx(ret, "uORB publications could not be unblocked");
			}

		} else {
			errx(1, "no valid command: %s", argv[1]);
		}
	}

	exit(0);
}

static void
do_gyro(int argc, char *argv[])
{
	int	fd;

	fd = open(argv[0], 0);

	if (fd < 0) {
		warn("%s", argv[0]);
		errx(1, "FATAL: no gyro found");

	} else {

		int ret;

		if (argc == 3 && !strcmp(argv[1], "sampling")) {

			/* set the gyro internal sampling rate up to at least i Hz */
			ret = ioctl(fd, GYROIOCSSAMPLERATE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "sampling rate could not be set");
			}

		} else if (argc == 3 && !strcmp(argv[1], "rate")) {

			/* set the driver to poll at i Hz */
			ret = ioctl(fd, SENSORIOCSPOLLRATE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "pollrate could not be set");
			}

		} else if (argc == 3 && !strcmp(argv[1], "range")) {

			/* set the range to i dps */
			ret = ioctl(fd, GYROIOCSRANGE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "range could not be set");
			}

		} else if (argc == 2 && !strcmp(argv[1], "check")) {
			ret = ioctl(fd, GYROIOCSELFTEST, 0);

			if (ret) {
				warnx("gyro self test FAILED! Check calibration:");
				struct gyro_calibration_s scale;
				ret = ioctl(fd, GYROIOCGSCALE, (long unsigned int)&scale);

				if (ret) {
					err(1, "failed getting gyro scale");
				}

				warnx("offsets: X: % 9.6f Y: % 9.6f Z: % 9.6f", (double)scale.x_offset, (double)scale.y_offset, (double)scale.z_offset);
				warnx("scale:   X: % 9.6f Y: % 9.6f Z: % 9.6f", (double)scale.x_scale, (double)scale.y_scale, (double)scale.z_scale);

			} else {
				warnx("gyro calibration and self test OK");
			}

		} else {
			errx(1, "wrong or no arguments given. Try: \n\n\t'check' for the self test\n\t'sampling 500' to set sampling to 500 Hz\n\t'rate 500' to set publication rate to 500 Hz\n\t'range 2000' to set measurement range to 2000 dps\n\t");
		}

		int srate = ioctl(fd, GYROIOCGSAMPLERATE, 0);
		int prate = ioctl(fd, SENSORIOCGPOLLRATE, 0);
		int range = ioctl(fd, GYROIOCGRANGE, 0);
		int id = ioctl(fd, DEVIOCGDEVICEID, 0);
		int32_t calibration_id = 0;

		param_get(param_find("CAL_GYRO0_ID"), &(calibration_id));

		warnx("gyro: \n\tdevice id:\t0x%X\t(calibration is for device id 0x%X)\n\tsample rate:\t%d Hz\n\tread rate:\t%d Hz\n\trange:\t%d dps",
		      id, calibration_id, srate, prate, range);

		close(fd);
	}

	exit(0);
}

static void
do_mag(int argc, char *argv[])
{
	int fd;

	fd = open(argv[0], 0);

	if (fd < 0) {
		warn("%s", argv[0]);
		errx(1, "FATAL: no magnetometer found");

	} else {

		int ret;

		if (argc == 3 && !strcmp(argv[1], "sampling")) {

			/* set the mag internal sampling rate up to at least i Hz */
			ret = ioctl(fd, MAGIOCSSAMPLERATE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "sampling rate could not be set");
			}

		} else if (argc == 3 && !strcmp(argv[1], "rate")) {

			/* set the driver to poll at i Hz */
			ret = ioctl(fd, SENSORIOCSPOLLRATE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "pollrate could not be set");
			}

		} else if (argc == 3 && !strcmp(argv[1], "range")) {

			/* set the range to i G */
			ret = ioctl(fd, MAGIOCSRANGE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "range could not be set");
			}

		} else if (argc == 2 && !strcmp(argv[1], "check")) {
			ret = ioctl(fd, MAGIOCSELFTEST, 0);

			if (ret) {
				warnx("mag self test FAILED! Check calibration:");
				struct mag_calibration_s scale;
				ret = ioctl(fd, MAGIOCGSCALE, (long unsigned int)&scale);

				if (ret) {
					err(ret, "failed getting mag scale");
				}

				warnx("offsets: X: % 9.6f Y: % 9.6f Z: % 9.6f", (double)scale.x_offset, (double)scale.y_offset, (double)scale.z_offset);
				warnx("scale:   X: % 9.6f Y: % 9.6f Z: % 9.6f", (double)scale.x_scale, (double)scale.y_scale, (double)scale.z_scale);

			} else {
				warnx("mag calibration and self test OK");
			}

		} else {
			errx(1, "wrong or no arguments given. Try: \n\n\t'check' for the self test\n\t");
		}

		int srate = ioctl(fd, MAGIOCGSAMPLERATE, 0);
		int prate = ioctl(fd, SENSORIOCGPOLLRATE, 0);
		int range = ioctl(fd, MAGIOCGRANGE, 0);
		int id = ioctl(fd, DEVIOCGDEVICEID, 0);
		int32_t calibration_id = 0;

		param_get(param_find("CAL_MAG0_ID"), &(calibration_id));

		warnx("mag: \n\tdevice id:\t0x%X\t(calibration is for device id 0x%X)\n\tsample rate:\t%d Hz\n\tread rate:\t%d Hz\n\trange:\t%d Ga",
		      id, calibration_id, srate, prate, range);

		close(fd);
	}

	exit(0);
}

static void
do_accel(int argc, char *argv[])
{
	int	fd;

	fd = open(argv[0], 0);

	if (fd < 0) {
		warn("%s", argv[0]);
		errx(1, "FATAL: no accelerometer found");

	} else {

		int ret;

		if (argc == 3 && !strcmp(argv[1], "sampling")) {

			/* set the accel internal sampling rate up to at least i Hz */
			ret = ioctl(fd, ACCELIOCSSAMPLERATE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "sampling rate could not be set");
			}

		} else if (argc == 3 && !strcmp(argv[1], "rate")) {

			/* set the driver to poll at i Hz */
			ret = ioctl(fd, SENSORIOCSPOLLRATE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "pollrate could not be set");
			}

		} else if (argc == 3 && !strcmp(argv[1], "range")) {

			/* set the range to i G */
			ret = ioctl(fd, ACCELIOCSRANGE, strtoul(argv[2], NULL, 0));

			if (ret) {
				errx(ret, "range could not be set");
			}

		} else if (argc == 2 && !strcmp(argv[1], "check")) {
			ret = ioctl(fd, ACCELIOCSELFTEST, 0);

			if (ret) {
				warnx("accel self test FAILED! Check calibration:");
				struct accel_calibration_s scale;
				ret = ioctl(fd, ACCELIOCGSCALE, (long unsigned int)&scale);

				if (ret) {
					err(ret, "failed getting accel scale");
				}

				warnx("offsets: X: % 9.6f Y: % 9.6f Z: % 9.6f", (double)scale.x_offset, (double)scale.y_offset, (double)scale.z_offset);
				warnx("scale:   X: % 9.6f Y: % 9.6f Z: % 9.6f", (double)scale.x_scale, (double)scale.y_scale, (double)scale.z_scale);

			} else {
				warnx("accel calibration and self test OK");
			}

		} else {
			errx(1, "no arguments given. Try: \n\n\t'sampling 500' to set sampling to 500 Hz\n\t'rate 500' to set publication rate to 500 Hz\n\t'range 4' to set measurement range to 4 G\n\t");
		}

		int srate = ioctl(fd, ACCELIOCGSAMPLERATE, 0);
		int prate = ioctl(fd, SENSORIOCGPOLLRATE, 0);
		int range = ioctl(fd, ACCELIOCGRANGE, 0);
		int id = ioctl(fd, DEVIOCGDEVICEID, 0);
		int32_t calibration_id = 0;

		param_get(param_find("CAL_ACC0_ID"), &(calibration_id));

		warnx("accel: \n\tdevice id:\t0x%X\t(calibration is for device id 0x%X)\n\tsample rate:\t%d Hz\n\tread rate:\t%d Hz\n\trange:\t%d G",
		      id, calibration_id, srate, prate, range);

		close(fd);
	}

	exit(0);
}
