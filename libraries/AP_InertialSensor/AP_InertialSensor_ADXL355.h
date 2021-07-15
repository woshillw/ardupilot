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
/*
  the ADIS1647x is unusual as it uses 16 bit registers. It also needs
  to run as the only sensor on the SPI bus for good performance
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADXL355 : public AP_InertialSensor_Backend
{
public:
  static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                          AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                                          enum Rotation rotation);

  /**
     * Configure the sensors and start reading routine.
     */
  void start() override;
  bool update() override;

private:
  AP_InertialSensor_ADXL355(AP_InertialSensor &imu,
                            AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                            enum Rotation rotation);

  /*
     initialise hardware layer
     */
  bool accel_init();

  /*
      initialise driver
     */
  bool init();

  /*
      read data from the FIFOs
     */
  void read_fifo_accel();

  uint8_t _register_read(uint8_t reg);
  /*
      write to an accelerometer register with retries
     */
  bool write_accel_register(uint8_t reg, uint8_t v);

  /*
      configure accel registers
     */
  bool setup_accel_config(void);

  AP_HAL::OwnPtr<AP_HAL::Device> dev_accel;

  uint8_t accel_instance;

  enum Rotation rotation;
  uint8_t temperature_counter;

  bool done_accel_config;
  uint32_t accel_config_count;
};
