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
 * @file lis3mdl_spi.cpp
 *
 * SPI interface for LIS3MDL
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

#include <drivers/device/spi.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "lis3mdl.h"
#include "board_config.h"

#ifdef PX4_SPIDEV_LIS

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

device::Device *LIS3MDL_SPI_interface(int bus);

class LIS3MDL_SPI : public device::SPI
{
public:
	LIS3MDL_SPI(int bus, spi_dev_e device);
	virtual ~LIS3MDL_SPI();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);
};

device::Device *
LIS3MDL_SPI_interface(int bus)
{
	return new LIS3MDL_SPI(bus, (spi_dev_e)PX4_SPIDEV_LIS);
}

LIS3MDL_SPI::LIS3MDL_SPI(int bus, spi_dev_e device) :
	SPI("LIS3MDL_SPI", nullptr, bus, device, SPIDEV_MODE3, 11 * 1000 * 1000 /* will be rounded to 10.4 MHz */)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;
}

LIS3MDL_SPI::~LIS3MDL_SPI()
{
}

int
LIS3MDL_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// read WHO_AM_I value
	uint8_t data = 0;

	if (read(ADDR_WHO_AM_I, &data, 1)) {
		DEVICE_DEBUG("LIS3MDL read_reg fail");
	}

	if (data != ID_WHO_AM_I) {
		DEVICE_DEBUG("LIS3MDL bad ID: %02x", data);
		return -EIO;
	}

	return OK;
}

int
LIS3MDL_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case MAGIOCGEXTERNAL:
		/*
		 * Even if this sensor is on the external SPI
		 * bus it is still internal to the autopilot
		 * assembly, so always return 0 for internal.
		 */
		return 0;

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

int
LIS3MDL_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}

int
LIS3MDL_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ | ADDR_INCREMENT;

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

#endif /* PX4_SPIDEV_LIS */
