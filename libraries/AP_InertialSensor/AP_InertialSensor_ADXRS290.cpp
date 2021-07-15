/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#include <utility>

#include "AP_InertialSensor_ADXRS290.h"

/* Gyroscope */
#define ADXRS290_ANALOG_ID 0x00
#define ADXRS290_ANALOG_ID_RETURN 0xAD

#define ADXRS290_MEMS_ID 0x01
#define ADXRS290_MEMS_ID_RETURN 0x1D

#define ADXRS290_DEV_ID 0x02
#define ADXRS290_DEV_ID_RETURN 0x92

#define ADXRS290_REV_NUM 0x03
#define ADXRS290_REV_NUM_RETURN 0x1D

#define ADXRS290_SERIALNUM_START 0x04
#define ADXRS290_SERIALNUM_END 0x07

// 16 2-complement's bits
#define ADXRS290_GYR_X_L 0x08
#define ADXRS290_GYR_X_H 0x09

// 16 2-complement's bits
#define ADXRS290_GYR_Y_L 0x0A
#define ADXRS290_GYR_Y_H 0x0B

// 12 2-complement's bits
#define ADXRS290_TEMP_L 0x0C		  // 7..0 bits
#define ADXRS290_TEMP_H 0x0D		  // 11..8 bits
#define ADXRS290_TEMP_SCALE_FACT 10.0 // 10 LSB /centig deg. 0 mean s o deg.

// The LSB controls Temperature sensor. 1 enable, 0 disable
#define ADXRS290_POW_CTRL_REG 0x10
#define ADXRS290_POW_CTRL_TEMP_EN_MASK 0x01

// 1: measurement mode, 0 chip in standby mode
#define ADXRS290_POW_CTRL_STDBY_MASK 0x02

// 1: measurement mode, 0 chip in standby mode
#define ADXRS290_BANDPASS_FILTER 0x11
#define ADXRS290_BPF_LPF_MASK 0x07
#define ADXRS290_BPF_HPF_MASK 0xF0
#define ADXRS290_BPF_HPF_OFFSET 0x4

#define ADXRS290_DATA_READY_REG 0x12

/* Set this bit to get triggered on data ready via interrupt
 * Set bit to 01 to gen rata ready interrupt
 * at the sync/asel pin when data becomes avail
 * Sync bits meaning:
 * X0 = Read for analog enable
 * 01 Data ready, high until read
 */
#define ADXRS290_DATA_READY_INT_MASK 0x03

/*Sensors Sensitivity */

/*
 * Low-Pass Filter Pole Locations
 *  The data is the Frequency in Hz
 */
#define ADXRS_LPF_480_HZ 0x00  //  480_HZ is Default
#define ADXRS_LPF_320_HZ 0x01  //  320_HZ
#define ADXRS_LPF_160_HZ 0x02  //  160_HZ
#define ADXRS_LPF_80_HZ 0x03   //  80_HZ
#define ADXRS_LPF_56_6_HZ 0x04 //  56.6_HZ
#define ADXRS_LPF_40_HZ 0x05   //  40 HZ
#define ADXRS_LPF_28_3_HZ 0x06 //  28.3_HZ
#define ADXRS_LPF_20_HZ 0x07   //  20_HZ

/*
 * High-Pass Filter Pole Locations
 *  The data is the Frequency in Hz
 */
#define ADXRS_HPF_ALL_HZ 0x00	// All Pass Default
#define ADXRS_HPF_0_011_HZ 0x01 //  0.011_Hz
#define ADXRS_HPF_0_022_HZ 0x02 //  0.022_Hz
#define ADXRS_HPF_0_044_HZ 0x03 //  0.044_Hz
#define ADXRS_HPF_0_087_HZ 0x04 //  0.087_Hz
#define ADXRS_HPF_0_175_HZ 0x05 //  0.187_Hz
#define ADXRS_HPF_0_350_HZ 0x06 //  0.350_Hz
#define ADXRS_HPF_0_700_HZ 0x07 //  0.700_Hz
#define ADXRS_HPF_1_400_HZ 0x08 //  1.400_Hz
#define ADXRS_HPF_2_800_HZ 0x09 //  2.800_Hz
#define ADXRS_HPF_11_30_HZ 0x0A // 11.300_Hz

#define gyro_raw_to_radian radians(1.0f / 200.0f)

#define GYRO_BACKEND_SAMPLE_RATE 2000

extern const AP_HAL::HAL &hal;

AP_InertialSensor_ADXRS290::AP_InertialSensor_ADXRS290(AP_InertialSensor &imu,
													   AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyroA,
													   AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyroB,
													   enum Rotation _rotationA,
													   enum Rotation _rotationB)
	: AP_InertialSensor_Backend(imu),
	  dev_gyro{std::move(_dev_gyroA), std::move(_dev_gyroB)},
	  rotation{_rotationA, _rotationB}
{
}

AP_InertialSensor_Backend *AP_InertialSensor_ADXRS290::probe(AP_InertialSensor &imu,
															 AP_HAL::OwnPtr<AP_HAL::Device> dev_gyroA,
															 AP_HAL::OwnPtr<AP_HAL::Device> dev_gyroB,
															 enum Rotation rotationA,
															 enum Rotation rotationB)

{
	if (!dev_gyroA || !dev_gyroB)
	{
		return nullptr;
	}
	auto sensor = new AP_InertialSensor_ADXRS290(imu,
												 std::move(dev_gyroA),
												 std::move(dev_gyroB),
												 rotationA,
												 rotationB);

	if (!sensor)
	{
		return nullptr;
	}

	if (!sensor->init())
	{
		delete sensor;
		return nullptr;
	}

	return sensor;
}

uint8_t AP_InertialSensor_ADXRS290::read_gyro_register(uint8_t num, uint8_t reg)
{
	uint8_t val = 0;

	dev_gyro[num]->read_registers(reg | 0x80, &val, 1);
	return val;
}

bool AP_InertialSensor_ADXRS290::write_gyro_register(uint8_t num, uint8_t reg, uint8_t v)
{

	for (uint8_t ii = 0; ii < 5; ii++)
	{
		dev_gyro[num]->write_register(reg & 0x7f, v);

		if (read_gyro_register(num, reg | 0x80) == v)
		{
			return true;
		}
	}
	return false;
}

bool AP_InertialSensor_ADXRS290::init()
{
	for (uint8_t i = 0; i < adxrs_num; i++)
		if (!gyro_init(i))
			return false;

	return true;
}

bool AP_InertialSensor_ADXRS290::gyro_init(uint8_t num)
{
	WITH_SEMAPHORE(dev_gyro[num]->get_semaphore());

	if (read_gyro_register(num, ADXRS290_ANALOG_ID) != ADXRS290_ANALOG_ID_RETURN)
	{
		return false;
	}

	if (!setup_gyro_config(num))
	{
		hal.console->printf("ADXRS290[%d]: delaying ADXRS290 config\n", num);
	}

	hal.console->printf("ADXRS290[%d]: found ADXRS290\n", num);

	return true;
}

void AP_InertialSensor_ADXRS290::start()
{
	if (!_imu.register_gyro(gyro_instance[0], GYRO_BACKEND_SAMPLE_RATE, dev_gyro[0]->get_bus_id_devtype(DEVTYPE_GYR_ADXRS290)))
	{
		return;
	}

	// if (!_imu.register_gyro(gyroB_instance, GYRO_BACKEND_SAMPLE_RATE, dev_gyroB->get_bus_id_devtype(DEVTYPE_GYR_ADXRS290)))
	// {
	// 	return;
	// }
	// setup sensor rotations from probe()
	set_gyro_orientation(gyro_instance[0], rotation[0]);
	// set_gyro_orientation(gyroB_instance, rotationB);

	// setup callbacks
	dev_gyro[0]->register_periodic_callback(1000000UL / GYRO_BACKEND_SAMPLE_RATE,
											FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADXRS290::_poll_data, void));
}

bool AP_InertialSensor_ADXRS290::setup_gyro_config(uint8_t num)
{
	if (done_gyro_config[num])
	{
		return true;
	}
	gyro_config_count[num]++;

	write_gyro_register(num, ADXRS290_DATA_READY_REG, 0);
	hal.scheduler->delay_microseconds(5);

	uint8_t res = read_gyro_register(num, ADXRS290_POW_CTRL_REG);
	write_gyro_register(num, ADXRS290_POW_CTRL_REG, res | (1 << 1));
	hal.scheduler->delay_microseconds(5);

	done_gyro_config[num] = true;
	hal.console->printf("ADXRS290a: ADXRS290a config OK (%u tries)\n", (unsigned)gyro_config_count[num]);
	return true;

	return false;
}

/*
  read accel fifo
 */
void AP_InertialSensor_ADXRS290::_poll_data(void)
{
	uint16_t temp;
	uint8_t raw_data_temp[4];

	const uint8_t _reg = ADXRS290_GYR_X_L | 0x80;

	// hal.console->printf("ADXRS290b\n");

	if (!dev_gyro[0]->read_registers(_reg, (uint8_t *)&raw_data_temp, sizeof(raw_data_temp)))
	{
		hal.console->printf("ADXRS290a: error reading ADXRS290a\n");
		return;
	}
	// hal.console->printf("ADXRS290: error reading ADXRS290\n");

	int dataA[2];
	temp = ((uint16_t)raw_data_temp[1] << 8) | raw_data_temp[0];
	dataA[0] = (int)((int16_t)temp);

	temp = ((uint16_t)raw_data_temp[3] << 8) | raw_data_temp[2];
	dataA[1] = (int)((int16_t)temp);

	if (!dev_gyro[1]->read_registers(_reg, (uint8_t *)&raw_data_temp, sizeof(raw_data_temp)))
	{
		hal.console->printf("ADXRS290b: error reading ADXRS290b\n");
		return;
	}

	int dataB[2];
	temp = ((uint16_t)raw_data_temp[1] << 8) | raw_data_temp[0];
	dataB[0] = (int)((int16_t)temp);

	temp = ((uint16_t)raw_data_temp[3] << 8) | raw_data_temp[2];
	dataB[1] = (int)((int16_t)temp);

	Vector3f gyro_data(-(float)dataA[0], -(float)dataB[0], (float)dataB[1]);
	gyro_data *= gyro_raw_to_radian;

	// hal.console->printf("ADXRS290: %f,%f,%f\n", gyro_data[0], gyro_data[1], gyro_data[2]);

	_rotate_and_correct_gyro(gyro_instance[0], gyro_data);
	_notify_new_gyro_raw_sample(gyro_instance[0], gyro_data);
}

bool AP_InertialSensor_ADXRS290::update()
{
	update_gyro(gyro_instance[0]);

	return true;
}
