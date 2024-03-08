/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulate EulerNav serial AHRS
*/

#include "SIM_EulerNav.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_Common/NMEA.h>
#include <AP_Math/crc.h>

using namespace SITL;

EulerNav::EulerNav() :
    SerialDevice::SerialDevice()
{
}

void EulerNav::send_EulerNav_inertial_output() {
    const auto &fdm = _sitl->state;

    EulerNav_inertial_output_data data;
    data.sequenceCounter = inertial_sequence_counter;
    data.specificForceX = fdm.xAccel;
    data.specificForceY = fdm.yAccel;
    data.specificForceZ = fdm.zAccel;
    data.angularRateX = radians(fdm.rollRate);
    data.angularRateY = radians(fdm.pitchRate);
    data.angularRateZ = radians(fdm.yawRate);

    data.validityFlags = 0xFF;

    inertial_sequence_counter++;
    inertial_time_us = AP_HAL::micros64();

    const auto len = sizeof(data);
    const auto padding_size = EulerNav_AlignTo32Bit(len);
    uint8_t padding[padding_size]{0};

    uint32_t crc = EULERNAV_INIT_CRC;
    crc = crc32_small(crc, (const uint8_t *)&data, len);
    crc = crc32_small(crc, (const uint8_t *)&padding, padding_size);

    write_to_autopilot((char *)&data, len);
    write_to_autopilot((char *)&padding, padding_size);
    write_to_autopilot((char *)&crc, sizeof(crc));
}


void EulerNav::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }
    // SoftwareVersion once at start
    //5ms : InertialData
// 10Hz : TimeOfInertialData
// 10ms : navigation data
// 10Hz: Accuracy data, TimeOfNavigationData
    const uint32_t ms_between_imu_packets = 5;
    uint32_t now = AP_HAL::millis();
    if (now - last_pkt1_us >= ms_between_imu_packets) {
        last_pkt1_us = now;
        send_EulerNav_inertial_output();
    }

}

