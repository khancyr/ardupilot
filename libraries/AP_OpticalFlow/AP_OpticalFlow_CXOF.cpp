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

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_OpticalFlow_CXOF.h"
#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the opticalflow. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the opticalflow
*/
AP_OpticalFlow_CXOF::AP_OpticalFlow_CXOF(OpticalFlow &_frontend, AP_SerialManager &serial_manager) :
        OpticalFlow_backend(_frontend)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, 0));
    }
}

/*
   detect if a cx-of opticalflow is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_OpticalFlow_CXOF::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_OpticalFlow_CXOF::get_reading(struct OpticalFlow::OpticalFlow_state& state)
{
    if (uart == nullptr) {
        return false;
    }

    bool pkt_valid = false;
    uint8_t received_char = 0;
    int32_t nbytes = uart->available();
    if (nbytes < 0) {
        return false;
    }

    while (nbytes-- > 0) {
        received_char = uart->read();
        if (pkt_count == 0) {
            if (received_char != 0xFE) {  // wait for header
                break;
            }
        }
        /* CXOF packet
        uint8_t     header;
        uint8_t     res0;
        int16_t     motionX;
        int16_t     motionY;
        int8_t      motionT;
        uint8_t     quality;
        uint8_t     footer;
         */
        if (pkt_count < CXOF_PACKET_SIZE) {
            input_buffer[pkt_count++] = received_char;
            if (pkt_count == CXOF_PACKET_SIZE) {
                if (input_buffer[0] == 0xFE && input_buffer[8] == 0xAA) {
                    // Valid packet
                    state.device_id = 1;
                    state.surface_quality = (constrain_value(input_buffer[7], (uint8_t)64, (uint8_t)78) - 64) * 100 / 14;
                    const Vector2f flowScaler = _flowScaler();
                    float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
                    float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
                    const uint16_t motionX = ((uint16_t) input_buffer[2] << 8) | input_buffer[3];
                    const uint16_t motionY = ((uint16_t) input_buffer[4] << 8) | input_buffer[5];
                    state.flowRate = Vector2f(motionX * flowScaleFactorX,
                                              motionY * flowScaleFactorY);
                    // we only apply yaw to flowRate as body rate comes from AHRS
                    _applyYaw(state.flowRate);
                    const Vector3f &gyro = AP::ahrs().get_gyro();
                    state.bodyRate = Vector2f(gyro.x, gyro.y);
                    pkt_valid = true;
                }
                // Reset the decoder
                pkt_count = 0;
            }
        } else {
            // In case of buffer overflow - reset the decoder
            pkt_count = 0;
        }
    }

    return pkt_valid;
}

/*
   update the state of the sensor
*/
void AP_OpticalFlow_CXOF::update()
{
    struct OpticalFlow::OpticalFlow_state state{};
    if (!get_reading(state)) {
        return;
    }
    _update_frontend(state);
}
