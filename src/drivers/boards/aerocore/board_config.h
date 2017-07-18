/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * AeroCore internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>


/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* LEDs */
#define GPIO_LED0	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN9)
#define GPIO_LED1	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)

/* Gyro */
#define GPIO_EXTI_GYRO_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN0)
#define SENSOR_BOARD_ROTATION_DEFAULT	3 /* SENSOR_BOARD_ROTATION_270_DEG */

/* Accel & Mag */
#define GPIO_EXTI_MAG_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN1)
#define GPIO_EXTI_ACCEL_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN2)

/* GPS */
#define GPIO_GPS_NRESET		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#define GPIO_GPS_TIMEPULSE	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN4)
#define GPS_DEFAULT_UART_PORT	"/dev/ttyS0"

/* SPI3--Sensors */
#define PX4_SPI_BUS_SENSORS	3
#define GPIO_SPI_CS_ACCEL_MAG	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI_CS_GYRO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_BARO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)

/* SPI4--Ramtron */
#define PX4_SPI_BUS_RAMTRON	4

/* Nominal chip selects for devices on SPI bus #3 */
#define PX4_SPIDEV_ACCEL_MAG	0
#define PX4_SPIDEV_GYRO		1
#define PX4_SPIDEV_BARO		2

/* User GPIOs broken out on J11 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO6_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO7_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO8_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO9_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15)
#define GPIO_GPIO10_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_GPIO11_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8)

#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO8_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO9_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_GPIO10_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define GPIO_GPIO11_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	((uint8_t)(-1))

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV   (7.8196363636f)
#define BOARD_BATTERY1_A_PER_V (15.391030303f)

/* PWM
 *
 * Eight PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PA8  : TIM1_CH1
 * CH2 : PA9  : TIM1_CH2
 * CH3 : PA10 : TIM1_CH3
 * CH4 : PA11 : TIM1_CH4
 * CH5 : PC6  : TIM3_CH1
 * CH6 : PC7  : TIM3_CH2
 * CH7 : PC8  : TIM3_CH3
 * CH8 : PC9  : TIM3_CH4
 */
#define GPIO_TIM1_CH1OUT	GPIO_TIM1_CH1OUT_1
#define GPIO_TIM1_CH2OUT	GPIO_TIM1_CH2OUT_1
#define GPIO_TIM1_CH3OUT	GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH4OUT	GPIO_TIM1_CH4OUT_1
#define GPIO_TIM3_CH1OUT	GPIO_TIM3_CH1OUT_3
#define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_3
#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_2
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_2
#define DIRECT_PWM_OUTPUT_CHANNELS	8

#define GPIO_TIM1_CH1IN		GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN		GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN		GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN		GPIO_TIM1_CH4IN_2
#define GPIO_TIM3_CH1IN		GPIO_TIM3_CH1IN_3
#define GPIO_TIM3_CH2IN		GPIO_TIM3_CH2IN_3
#define GPIO_TIM3_CH3IN		GPIO_TIM3_CH3IN_2
#define GPIO_TIM3_CH4IN		GPIO_TIM3_CH4IN_2
#define DIRECT_INPUT_TIMER_CHANNELS	8

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer 8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

/* Tone Alarm (no onboard speaker )*/
#define TONE_ALARM_TIMER	2	/* timer 2 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN0)

#define BOARD_NAME "AEROCORE"

#define BOARD_HAS_PWM	8
/* AeroCore breaks out User GPIOs on J11 */
#define BOARD_FMU_GPIO_TAB  { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, \
		{GPIO_GPIO6_INPUT,       GPIO_GPIO6_OUTPUT,       0}, \
		{GPIO_GPIO7_INPUT,       GPIO_GPIO7_OUTPUT,       0}, \
		{GPIO_GPIO8_INPUT,       GPIO_GPIO8_OUTPUT,       0}, \
		{GPIO_GPIO9_INPUT,       GPIO_GPIO9_OUTPUT,       0}, \
		{GPIO_GPIO10_INPUT,      GPIO_GPIO10_OUTPUT,      0}, \
		{GPIO_GPIO11_INPUT,      GPIO_GPIO11_OUTPUT,      0}, }

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1             (1<<0)   /**< servo 1 output */
#define GPIO_SERVO_2             (1<<1)   /**< servo 2 output */
#define GPIO_SERVO_3             (1<<2)   /**< servo 3 output */
#define GPIO_SERVO_4             (1<<3)   /**< servo 4 output */
#define GPIO_SERVO_5             (1<<4)   /**< servo 5 output */
#define GPIO_SERVO_6             (1<<5)   /**< servo 6 output */
#define GPIO_SERVO_7             (1<<6)   /**< servo 7 output */
#define GPIO_SERVO_8             (1<<6)   /**< servo 8 output */
#define GPIO_SERVO_9             (1<<8)   /**< servo 9 output */
#define GPIO_SERVO_10            (1<<9)   /**< servo 10 output */
#define GPIO_SERVO_11            (1<<10)  /**< servo 11 output */
#define GPIO_SERVO_12            (1<<11)  /**< servo 12 output */

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);
#define board_spi_reset(ms)

#define board_peripheral_reset(ms)

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
