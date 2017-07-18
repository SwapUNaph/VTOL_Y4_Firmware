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
 * @file HMC5883_I2C.cpp
 *
 * I2C interface for HMC5883 / HMC 5983
 */

/* XXX trim includes */
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "hmc5883.h"

#include "board_config.h"

#ifdef PX4_I2C_OBDEV_HMC5883

#define HMC5883L_ADDRESS		PX4_I2C_OBDEV_HMC5883

device::Device *HMC5883_I2C_interface(int bus);

class HMC5883_I2C : public device::I2C
{
public:
	HMC5883_I2C(int bus);
	virtual ~HMC5883_I2C();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	virtual int	probe();

};

device::Device *
HMC5883_I2C_interface(int bus)
{
	return new HMC5883_I2C(bus);
}

HMC5883_I2C::HMC5883_I2C(int bus) :
	I2C("HMC5883_I2C", nullptr, bus, HMC5883L_ADDRESS, 400000)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_HMC5883;
}

HMC5883_I2C::~HMC5883_I2C()
{
}

int
HMC5883_I2C::init()
{
	/* this will call probe() */
	return I2C::init();
}

int
HMC5883_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case MAGIOCGEXTERNAL:
#ifdef PX4_I2C_BUS_ONBOARD
		if (_bus == PX4_I2C_BUS_ONBOARD) {
			return 0;

		} else {
			return 1;
		}

#else
		/* assume external for all boards that don't define PX4_I2C_BUS_ONBOARD */
		return 1;
#endif

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default:
		ret = -EINVAL;
	}

	return ret;
}

int
HMC5883_I2C::probe()
{
	uint8_t data[3] = {0, 0, 0};

	_retries = 10;

	if (read(ADDR_ID_A, &data[0], 1) ||
	    read(ADDR_ID_B, &data[1], 1) ||
	    read(ADDR_ID_C, &data[2], 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if ((data[0] != ID_A_WHO_AM_I) ||
	    (data[1] != ID_B_WHO_AM_I) ||
	    (data[2] != ID_C_WHO_AM_I)) {
		DEVICE_DEBUG("ID byte mismatch (%02x,%02x,%02x)", data[0], data[1], data[2]);
		return -EIO;
	}

	return OK;
}

int
HMC5883_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int
HMC5883_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

#endif /* PX4_I2C_OBDEV_HMC5883 */
