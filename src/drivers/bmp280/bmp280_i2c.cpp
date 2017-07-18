/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file bmp280_spi.cpp
 *
 * SPI interface for BMP280
 */

#include <px4_config.h>

#include <drivers/bmp280/bmp280.h>
#include <drivers/device/i2c.h>

#include "board_config.h"


#if defined(PX4_I2C_OBDEV_BMP280) || defined(PX4_I2C_EXT_OBDEV_BMP280)


class BMP280_I2C: public device::I2C, public bmp280::IBMP280
{
public:
	BMP280_I2C(uint8_t bus, uint8_t device, bool external);
	~BMP280_I2C();

	bool is_external();
	int init();

	uint8_t get_reg(uint8_t addr);
	int set_reg(uint8_t value, uint8_t addr);
	bmp280::data_s *get_data(uint8_t addr);
	bmp280::calibration_s *get_calibration(uint8_t addr);

private:
	struct bmp280::calibration_s _cal;
	struct bmp280::data_s _data;
	bool _external;
};

bmp280::IBMP280 *bmp280_i2c_interface(uint8_t busnum, uint8_t device, bool external)
{
	return new BMP280_I2C(busnum, device, external);
}

BMP280_I2C::BMP280_I2C(uint8_t bus, uint8_t device, bool external) :
	I2C("BMP280_I2C", nullptr, bus, device, 100 * 1000)
{
	_external = external;
}


BMP280_I2C::~BMP280_I2C()
{
}


bool BMP280_I2C::is_external()
{
	return _external;
}

int BMP280_I2C::init()
{
	return I2C::init();
}

uint8_t BMP280_I2C::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), 0};
	transfer(&cmd[0], 1, &cmd[1], 1);

	return cmd[1];

}

int BMP280_I2C::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);

}

bmp280::data_s *BMP280_I2C::get_data(uint8_t addr)
{
	const uint8_t cmd = (uint8_t)(addr);

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_data, sizeof(struct bmp280::data_s)) == OK) {
		return (&_data);

	} else {
		return nullptr;
	}


}

bmp280::calibration_s *BMP280_I2C::get_calibration(uint8_t addr)
{
	const uint8_t cmd = (uint8_t)(addr) ;

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_cal, sizeof(struct bmp280::calibration_s)) == OK) {
		return &(_cal);

	} else {
		return nullptr;
	}
}



#endif /* PX4_I2C_OBDEV_BMP280 || PX4_I2C_EXT_OBDEV_BMP280 */
