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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_InertialSensor_ADXL355.h"

#define ADXL355_DEVID_AD_REG 0x00
#define ADXL355_DEVID_MST_REG 0x01
#define ADXL355_PARTID_REG 0x02
#define ADXL355_REVID_REG 0x03
#define ADXL355_STATUS_REG 0x04
#define ADXL355_FIFO_ENTRIES_REG 0x05
#define ADXL355_TEMP2_REG 0x06
#define ADXL355_TEMP1_REG 0x07
#define ADXL355_XDATA3_REG 0x08
#define ADXL355_XDATA2_REG 0x09
#define ADXL355_XDATA1_REG 0x0A
#define ADXL355_YDATA3_REG 0x0B
#define ADXL355_YDATA2_REG 0x0C
#define ADXL355_YDATA1_REG 0x0D
#define ADXL355_ZDATA3_REG 0x0E
#define ADXL355_ZDATA2_REG 0x0F
#define ADXL355_ZDATA1_REG 0x10
#define ADXL355_FIFO_DATA_REG 0x11

////REG WriteOnly
#define ADXL355_RST_REG 0x2F

////REG ReadWrite
#define ADXL355_OFFSET_X_H_REG 0x1E
#define ADXL355_OFFSET_X_L_REG 0x1F
#define ADXL355_OFFSET_Y_H_REG 0x20
#define ADXL355_OFFSET_Y_L_REG 0x21
#define ADXL355_OFFSET_Z_H_REG 0x22
#define ADXL355_OFFSET_Z_L_REG 0x23
#define ADXL355_ACT_EN_REG 0x24
#define ADXL355_ACT_THRESH_H_REG 0x25
#define ADXL355_ACT_THRESH_L_REG 0x26
#define ADXL355_ACT_COUNT_REG 0x27
#define ADXL355_FILTER_REG 0x28
#define ADXL355_FIFO_SAMPLES_REG 0x29
#define ADXL355_INT_MAP_REG 0x2A
#define ADXL355_Sync_REG 0x2B
#define ADXL355_Range_REG 0x2C
#define ADXL355_POWER_CTL_REG 0x2D
#define ADXL355_SELF_TEST_REG 0x2E

///REG Value
#define ADXL355_DEVID_AD_RETURN 0xAD
#define ADXL355_DEVID_MST_RETURN 0x1D
#define ADXL355_PARTID_RETURN 0xED
#define ADXL355_REVID_RETURN 0x01
#define ADXL355_RST_VALUE 0x52

#define ADXL355_ACCELEROMETER_BW_RATE_4HZ 0x0A
#define ADXL355_ACCELEROMETER_BW_RATE_8HZ 0x09
#define ADXL355_ACCELEROMETER_BW_RATE_16HZ 0x08
#define ADXL355_ACCELEROMETER_BW_RATE_31HZ 0x07
#define ADXL355_ACCELEROMETER_BW_RATE_62_5HZ 0x06
#define ADXL355_ACCELEROMETER_BW_RATE_125HZ 0x05
#define ADXL355_ACCELEROMETER_BW_RATE_250HZ 0x04
#define ADXL355_ACCELEROMETER_BW_RATE_500HZ 0x03
#define ADXL355_ACCELEROMETER_BW_RATE_1000HZ 0x02
#define ADXL355_ACCELEROMETER_BW_RATE_2000HZ 0x01
#define ADXL355_ACCELEROMETER_BW_RATE_4000HZ 0x00

#define ADXL355_ACCELEROMETER_RANGE_2G 0x00
#define ADXL355_ACCELEROMETER_RANGE_4G 0x01
#define ADXL355_ACCELEROMETER_RANGE_8G 0x02

#define ACCEL_BACKEND_SAMPLE_RATE 1600

extern const AP_HAL::HAL &hal;

AP_InertialSensor_ADXL355::AP_InertialSensor_ADXL355(AP_InertialSensor &imu,
													 AP_HAL::OwnPtr<AP_HAL::Device> _dev_accel,
													 enum Rotation _rotation)
	: AP_InertialSensor_Backend(imu), dev_accel(std::move(_dev_accel)), rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADXL355::probe(AP_InertialSensor &imu,
								 AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
								 enum Rotation rotation)
{
	if (!dev_accel)
	{
		return nullptr;
	}
	auto sensor = new AP_InertialSensor_ADXL355(imu, std::move(dev_accel), rotation);

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

void AP_InertialSensor_ADXL355::start()
{
	if (!_imu.register_accel(accel_instance, ACCEL_BACKEND_SAMPLE_RATE, dev_accel->get_bus_id_devtype(DEVTYPE_ACC_ADXL355)))
	{
		return;
	}

	// setup sensor rotations from probe()
	set_accel_orientation(accel_instance, rotation);

	// setup callbacks
	dev_accel->register_periodic_callback(1000000UL / ACCEL_BACKEND_SAMPLE_RATE,
										  FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADXL355::read_fifo_accel, void));
}

/*
  read from accelerometer registers, special SPI handling needed
*/
uint8_t AP_InertialSensor_ADXL355::_register_read(uint8_t reg)
{
	uint8_t val = 0;
	dev_accel->read_registers((reg << 1) | 0x01, &val, 1);
	return val;
}

/*
  write to accel registers with retries. The SPI sensor may take
  several tries to correctly write a register
*/
bool AP_InertialSensor_ADXL355::write_accel_register(uint8_t reg, uint8_t v)
{
	for (uint8_t i = 0; i < 5; i++)
	{
		dev_accel->write_register(reg << 1, v);

		if (_register_read(reg) == v)
		{
			return true;
		}
	}
	return false;
}

bool AP_InertialSensor_ADXL355::setup_accel_config(void)
{
	hal.console->printf("ADXL355: setup_accel_config\n");

	if (done_accel_config)
	{
		return true;
	}
	accel_config_count++;

	write_accel_register(ADXL355_Range_REG, ADXL355_ACCELEROMETER_RANGE_2G);
	hal.scheduler->delay_microseconds(5);

	write_accel_register(ADXL355_FILTER_REG, ADXL355_ACCELEROMETER_BW_RATE_500HZ);
	hal.scheduler->delay_microseconds(5);

	write_accel_register(ADXL355_POWER_CTL_REG, 0);
	hal.scheduler->delay_microseconds(5);

	done_accel_config = true;
	hal.console->printf("ADXL355: accel config OK (%u tries)\n", (unsigned)accel_config_count);
	return true;
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_ADXL355::accel_init()
{
	WITH_SEMAPHORE(dev_accel->get_semaphore());

	if (_register_read(ADXL355_DEVID_AD_REG) != ADXL355_DEVID_AD_RETURN)
	{
		hal.console->printf("ADXL355: accel_init\n");
		return false;
	}

	if (!setup_accel_config())
	{
		hal.console->printf("ADXL355: delaying accel config\n");
	}

	hal.console->printf("ADXL355: found accel\n");

	return true;
}

bool AP_InertialSensor_ADXL355::init()
{
	return accel_init();
}

/*
  read accel fifo
 */
void AP_InertialSensor_ADXL355::read_fifo_accel(void)
{
	uint8_t raw_data_temp[9];

	const uint8_t _reg = (ADXL355_XDATA3_REG << 1) | 0x01;

	const float _accel_scale = (1.0f / 256000.0f) * GRAVITY_MSS;

	if (!dev_accel->read_registers(_reg, (uint8_t *)&raw_data_temp, sizeof(raw_data_temp)))
	{
		hal.console->printf("ADXL355: error reading accelerometer\n");
		return;
	}

	uint32_t a;
	uint16_t b;
	uint8_t c;
	int32_t valuex, valuey, valuez;
	int acc[3];

	a = (uint32_t)raw_data_temp[0] << 16;
	b = raw_data_temp[1] << 8;
	c = raw_data_temp[2] & 0xF0;
	valuex = a | b | c;
	*acc = (valuex << 8) >> 12; //带符号位扩展

	a = (uint32_t)raw_data_temp[3] << 16;
	b = raw_data_temp[4] << 8;
	c = raw_data_temp[5] & 0xF0;
	valuey = a | b | c;
	*(acc + 1) = (valuey << 8) >> 12; //带符号位扩展

	a = (uint32_t)raw_data_temp[6] << 16;
	b = raw_data_temp[7] << 8;
	c = raw_data_temp[8] & 0xF0;
	valuez = a | b | c;
	*(acc + 2) = (valuez << 8) >> 12; //带符号位扩展

	Vector3f accel_data(-(float)acc[1], -(float)acc[0], -(float)acc[2]);
	accel_data *= _accel_scale;

	// hal.console->printf("ADXL355: %f,%f,%f\n", accel_data[0], accel_data[1], accel_data[2]);

	_rotate_and_correct_accel(accel_instance, accel_data);
	_notify_new_accel_raw_sample(accel_instance, accel_data);
}

bool AP_InertialSensor_ADXL355::update()
{
	update_accel(accel_instance);

	return true;
}
