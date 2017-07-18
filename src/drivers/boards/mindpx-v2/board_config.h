/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 PX4 Development Team. All rights reserved.
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
 * MINDPXv2 internal definitions
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
/* Configuration ************************************************************************************/


/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/* External interrupts */
#define GPIO_EXTI_GYRO_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN4)
#define GPIO_EXTI_MAG_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)
#define GPIO_EXTI_ACCEL_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_EXTI_MPU_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN10)

/* Data ready pins off */
#define GPIO_GYRO_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTE|GPIO_PIN14)
#define GPIO_MAG_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTC|GPIO_PIN14)
#define GPIO_ACCEL_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTC|GPIO_PIN13)
#define GPIO_EXTI_MPU_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTE|GPIO_PIN10)


/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN5)
#define GPIO_SPI1_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN6)
#define GPIO_SPI1_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN5)

/*SPI1 chip selects off */
#define GPIO_SPI_CS_FRAM_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTE|GPIO_PIN12)

/* SPI1 chip selects */
#define GPIO_SPI_CS_FRAM	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)


/* SPI4 off */
#define GPIO_SPI4_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI4_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN5)
#define GPIO_SPI4_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN6)

/* SPI4 chip selects off */
#define GPIO_SPI_CS_GYRO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTB|GPIO_PIN2)
#define GPIO_SPI_CS_ACCEL_MAG_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTD|GPIO_PIN11)
#define GPIO_SPI_CS_BARO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_MPU_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTE|GPIO_PIN3)

/* SPI4 chip selects */
#define GPIO_SPI_CS_GYRO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define GPIO_SPI_CS_ACCEL_MAG	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define GPIO_SPI_CS_BARO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_MPU		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)


/* SPI2 off */
#define GPIO_SPI2_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN13)
#define GPIO_SPI2_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI2_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN15)

/* SPI2 chip selects off */
#define GPIO_SPI_CS_EXT0_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_25MHz|GPIO_PORTD|GPIO_PIN7)

/* SPI2 chip selects */
#define GPIO_SPI_CS_EXT0	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)

#define PX4_SPI_BUS_SENSORS	4
#define PX4_SPI_BUS_RAMTRON	1
#define PX4_SPI_BUS_EXT		2
#define PX4_SPI_BUS_BARO	PX4_SPI_BUS_SENSORS

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI4 */
#define PX4_SPIDEV_GYRO		1
#define PX4_SPIDEV_ACCEL_MAG	2
#define PX4_SPIDEV_BARO		3
#define PX4_SPIDEV_MPU		4

/* External bus */
#define PX4_SPIDEV_EXT0		1


/* I2C busses */
#define PX4_I2C_BUS_ONBOARD	1
#define PX4_I2C_BUS_EXPANSION	2
#define PX4_I2C_BUS_LED		PX4_I2C_BUS_EXPANSION

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_HMC5883	0x1e
#define PX4_I2C_OBDEV_LED	0x55
// #define PX4_I2C_OBDEV_MPU6050	0x68

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_5V_RAIL_SENSE		4
#define ADC_BATTERY_CURRENT_CHANNEL	10
#define ADC_BATTERY_VOLTAGE_CHANNEL	12
#define ADC_RC_RSSI_CHANNEL		11
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV   (10.177939394f)
#define BOARD_BATTERY1_A_PER_V (15.391030303f)

/* User GPIOs
 *
 * GPIO0-7 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO2_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO6_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO7_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15)


#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO2_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* Power supply control and monitoring GPIOs */
// #define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
// #define GPIO_VDD_BRICK_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
// #define GPIO_VDD_SERVO_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN7)
// #define GPIO_VDD_3V3_SENSORS_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
// #define GPIO_VDD_5V_HIPOWER_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
// #define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)

/* Tone alarm output */
#define TONE_ALARM_TIMER	14	/* timer 14 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF9|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN7)

/* AUX PWMs
 *
 * 8 PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PE9  : TIM1_CH1
 * CH2 : PE11 : TIM1_CH2
 * CH3 : PE13 : TIM1_CH3
 * CH4 : PE14 : TIM1_CH4
 * CH5 : PD12 : TIM4_CH1
 * CH6 : PD13 : TIM4_CH2
 * CH7 : PD14 : TIM4_CH3
 * CH8 : PD15 : TIM4_CH4
 */
#define GPIO_TIM1_CH1OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM1_CH3OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
#define GPIO_TIM1_CH4OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
#define GPIO_TIM4_CH1OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN12)
#define GPIO_TIM4_CH2OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM4_CH3OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN14)
#define GPIO_TIM4_CH4OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN15)
#define DIRECT_PWM_OUTPUT_CHANNELS	8

#define GPIO_TIM1_CH1IN		GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN		GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN		GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN		GPIO_TIM1_CH4IN_2
#define GPIO_TIM4_CH1IN		GPIO_TIM4_CH1IN_2
#define GPIO_TIM4_CH2IN		GPIO_TIM4_CH2IN_2
#define GPIO_TIM4_CH3IN		GPIO_TIM4_CH3IN_2
#define GPIO_TIM4_CH4IN		GPIO_TIM4_CH4IN_2

#define DIRECT_INPUT_TIMER_CHANNELS	8


/* PWMs for main output */
/* TODO: implement a new driver - /dev/pwm_output0 */
#define GPIO_TIM2_CH1OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TIM2_CH2OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN3)
#define GPIO_TIM2_CH3OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM2_CH4OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN3)
#define GPIO_TIM3_CH1OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN4)
#define GPIO_TIM3_CH2OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN7)
#define GPIO_TIM3_CH3OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN0)
#define GPIO_TIM3_CH4OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN1)

#define GPIO_TIM2_CH1IN		GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN		GPIO_TIM2_CH2IN_2
#define GPIO_TIM2_CH3IN		GPIO_TIM2_CH3IN_1
#define GPIO_TIM2_CH4IN		GPIO_TIM2_CH4IN_1
#define GPIO_TIM3_CH1IN		GPIO_TIM3_CH1IN_2
#define GPIO_TIM3_CH2IN		GPIO_TIM3_CH2IN_3
#define GPIO_TIM3_CH3IN		GPIO_TIM3_CH3IN_1
#define GPIO_TIM3_CH4IN		GPIO_TIM3_CH4IN_1


/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)



/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel */

#define HRT_PPM_CHANNEL		1
#define GPIO_PPM_IN		GPIO_TIM8_CH1IN_1

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 1 */
#define PWMIN_TIMER		4
#define PWMIN_TIMER_CHANNEL	1
#define GPIO_PWM_IN		GPIO_TIM4_CH1IN

#define RC_SERIAL_PORT		"/dev/ttyS0"

// #define GPIO_RSSI_IN                (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN1)
#define GPIO_SBUS_INV                  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)
#define INVERT_RC_INPUT(_invert_true)  px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true);

#define GPIO_FRSKY_INV                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define INVERT_FRSKY(_invert_true)    px4_arch_gpiowrite(GPIO_FRSKY_INV, _invert_true);

/* Power switch controls */
#define SPEKTRUM_POWER(_on_true)      do { } while (0)

/*
 * MindPXv2 has one RC_IN
 *
 * GPIO PPM_IN on PC6 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on PC6 UART1
 * Inversion is possible via the 74LVC2G86 controlled by the FMU
 * The FMU can drive GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()  px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()         px4_arch_configgpio(GPIO_USART1_RX)
#define SPEKTRUM_OUT(_one_true)       px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

#define BOARD_NAME "MINDPX_V2"

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB  { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, \
		{GPIO_GPIO6_INPUT,       GPIO_GPIO6_OUTPUT,       0}, \
		{GPIO_GPIO7_INPUT,       GPIO_GPIO7_OUTPUT,       0}, }

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1			(1<<0)		/**< servo 1 output */
#define GPIO_SERVO_2			(1<<1)		/**< servo 2 output */
#define GPIO_SERVO_3			(1<<2)		/**< servo 3 output */
#define GPIO_SERVO_4			(1<<3)		/**< servo 4 output */
#define GPIO_SERVO_5			(1<<4)		/**< servo 5 output */
#define GPIO_SERVO_6			(1<<5)		/**< servo 6 output */
#define GPIO_SERVO_7			(1<<6)		/**< servo 7 output */
#define GPIO_SERVO_8			(1<<7)		/**< servo 8 output */

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

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
void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);


#define board_peripheral_reset(ms)


#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
