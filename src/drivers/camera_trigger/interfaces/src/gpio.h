/**
 * @file relay.h
 *
 * Interface with cameras via FMU auxiliary pins.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <board_config.h>

#include "camera_interface.h"

class CameraInterfaceGPIO : public CameraInterface
{
public:
	CameraInterfaceGPIO();
	virtual ~CameraInterfaceGPIO();

	void trigger(bool enable);

	void info();

private:

	void setup();

	param_t _p_polarity;

	int _polarity;

	static constexpr uint32_t _gpios[6] = {
		GPIO_GPIO0_OUTPUT,
		GPIO_GPIO1_OUTPUT,
		GPIO_GPIO2_OUTPUT,
		GPIO_GPIO3_OUTPUT,
		GPIO_GPIO4_OUTPUT,
		GPIO_GPIO5_OUTPUT
	};

};

#endif /* ifdef __PX4_NUTTX */
