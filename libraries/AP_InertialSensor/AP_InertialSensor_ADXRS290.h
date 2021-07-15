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
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <Filter/LowPassFilter2p.h>

class AP_InertialSensor_ADXRS290 : public AP_InertialSensor_Backend
{
public:
   virtual ~AP_InertialSensor_ADXRS290() {}

   void start() override;
   bool update() override;

   static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                           AP_HAL::OwnPtr<AP_HAL::Device> dev_gyroA,
                                           AP_HAL::OwnPtr<AP_HAL::Device> dev_gyroB,
                                           enum Rotation rotationA,
                                           enum Rotation rotationB);

private:
   AP_InertialSensor_ADXRS290(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyroA,
                              AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyroB,
                              enum Rotation _rotationA,
                              enum Rotation _rotationB);

   enum
   {
      adxrs_a = 0,
      adxrs_b = 1,

      adxrs_num
   };

   bool init();

   bool gyro_init(uint8_t num);

   void _poll_data();

   uint8_t read_gyro_register(uint8_t num, uint8_t reg);

   bool write_gyro_register(uint8_t num, uint8_t reg, uint8_t v);

   /*
      configure accel registers
     */
   bool setup_gyro_config(uint8_t num);

   AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro[adxrs_num];

   uint8_t gyro_instance[adxrs_num];

   enum Rotation rotation[adxrs_num];

   bool done_gyro_config[adxrs_num];

   uint32_t gyro_config_count[adxrs_num];
};
