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

#include <stdint.h>

#include <systemlib/perf_counter.h>
#include <systemlib/conversions.h>

#include <nuttx/wqueue.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "gyro.h"



#if defined(PX4_I2C_OBDEV_MPU9250)
#  define USE_I2C
#endif


// MPU 9250 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_ACCEL_CONFIG2_41HZ		0x03

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_INT_BYPASS_EN		0x02

#define BIT_I2C_READ_FLAG           0x80

#define BIT_I2C_SLV0_NACK           0x01
#define BIT_I2C_FIFO_EN             0x40
#define BIT_I2C_MST_EN              0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_FIFO_RST                0x04
#define BIT_I2C_MST_RST             0x02
#define BIT_SIG_COND_RST            0x01

#define BIT_I2C_SLV0_EN             0x80
#define BIT_I2C_SLV0_BYTE_SW        0x40
#define BIT_I2C_SLV0_REG_DIS        0x20
#define BIT_I2C_SLV0_REG_GRP        0x10

#define BIT_I2C_MST_MULT_MST_EN     0x80
#define BIT_I2C_MST_WAIT_FOR_ES     0x40
#define BIT_I2C_MST_SLV_3_FIFO_EN   0x20
#define BIT_I2C_MST_P_NSR           0x10
#define BITS_I2C_MST_CLOCK_258HZ    0x08
#define BITS_I2C_MST_CLOCK_400HZ    0x0D

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08

#define MPU_WHOAMI_9250			0x71
#define MPU_WHOAMI_6500			0x70

#define MPU9250_ACCEL_DEFAULT_RATE	1000
#define MPU9250_ACCEL_MAX_OUTPUT_RATE			280
#define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define MPU9250_GYRO_DEFAULT_RATE	1000
/* rates need to be the same between accel and gyro */
#define MPU9250_GYRO_MAX_OUTPUT_RATE			MPU9250_ACCEL_MAX_OUTPUT_RATE
#define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define MPU9250_DEFAULT_ONCHIP_FILTER_FREQ	41

#define MPU9250_ONE_G					9.80665f

#define MPUIOCGIS_I2C	(unsigned)(DEVIOCGDEVICEID+100)


#pragma pack(push, 1)
/**
 * Report conversation within the mpu, including command byte and
 * interrupt status.
 */
struct MPUReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
	struct ak8963_regs mag;
};
#pragma pack(pop)

#define MPU_MAX_WRITE_BUFFER_SIZE (2)


/*
  The MPU9250 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using either
  I2C at 400kHz or SPI at 1MHz. For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */
#define MPU9250_LOW_BUS_SPEED				0
#define MPU9250_HIGH_BUS_SPEED				0x8000
#  define MPU9250_IS_HIGH_SPEED(r) 			((r) & MPU9250_HIGH_BUS_SPEED)
#  define MPU9250_REG(r) 					((r) &~MPU9250_HIGH_BUS_SPEED)
#  define MPU9250_SET_SPEED(r, s) 			((r)|(s))
#  define MPU9250_HIGH_SPEED_OP(r) 			MPU9250_SET_SPEED((r), MPU9250_HIGH_BUS_SPEED)
#  define MPU9250_LOW_SPEED_OP(r)			MPU9250_REG((r))

/* interface factories */
extern device::Device *MPU9250_SPI_interface(int bus, bool external_bus);
extern device::Device *MPU9250_I2C_interface(int bus, bool external_bus);
extern int MPU9250_probe(device::Device *dev, int device_type);

typedef device::Device *(*MPU9250_constructor)(int, bool);




class MPU9250_mag;
class MPU9250_gyro;

class MPU9250 : public device::CDev
{
public:
	MPU9250(device::Device *interface, device::Device *mag_interface, const char *path_accel, const char *path_gyro,
		const char *path_mag,
		enum Rotation rotation);
	virtual ~MPU9250();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

	// deliberately cause a sensor error
	void 			test_error();

protected:
	Device			*_interface;

	virtual int		probe();

	friend class MPU9250_mag;
	friend class MPU9250_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	MPU9250_gyro	*_gyro;
	MPU9250_mag     *_mag;
	uint8_t			_whoami;	/** whoami result */

#if defined(USE_I2C)
	/*
	 * SPI bus based device use hrt
	 * I2C bus needs to use work queue
	 */
	work_s			_work;
#endif
	bool 			_use_hrt;

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	ringbuffer::RingBuffer	*_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;

	unsigned		_dlpf_freq;

	unsigned		_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_duplicates;
	perf_counter_t		_controller_latency_perf;

	uint8_t			_register_wait;
	uint64_t		_reset_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	Integrator		_accel_int;
	Integrator		_gyro_int;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MPU9250_NUM_CHECKED_REGISTERS 11
	static const uint8_t	_checked_registers[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_bad[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// last temperature reading for print_info()
	float			_last_temperature;

	bool check_null_data(uint32_t *data, uint8_t size);
	bool check_duplicate(uint8_t *accel_data);
	// keep last accel reading for duplicate detection
	uint8_t			_last_accel_data[6];
	bool			_got_duplicate;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();



#if defined(USE_I2C)
	/**
	 * When the I2C interfase is on
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
		 * previous measurement.
		 *
		 * When the interval between measurements is greater than the minimum
		 * measurement interval, a gap is inserted between collection
		 * and measurement to provide the most recent measurement possible
		 * at the next interval.
		 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	void use_i2c(bool on_true) { _use_hrt = !on_true; }

#endif

	bool is_i2c(void) { return !_use_hrt; }




	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	/**
	 * Read a register from the mpu
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed = MPU9250_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the mpu
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the mpu, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the mpu measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the mpu to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external()
	{
		unsigned dummy;
		return _interface->ioctl(ACCELIOCGEXTERNAL, dummy);
	}

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			gyro_self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	MPU9250(const MPU9250 &);
	MPU9250 operator=(const MPU9250 &);
};
