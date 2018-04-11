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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_MotorController_Backend;

class AP_MotorController {
  public:
    friend class AP_MotorController_Backend;

    AP_MotorController();

    // external position backend types (used by _TYPE parameter)
    enum MotorController_Type {
        MotorController_TYPE_NONE = 0,
        MotorController_TYPE_ROBOCLAW = 1,
    };

    enum MotorController_Control {
        CONTROL_PWM = 0,
        CONTROL_SPEED = 1,
        CONTROL_POSITION = 2,
    };

    // initialise any available position estimators
    void init(void);

    // update state of all beacons
    void update(uint32_t motor1, uint32_t motor2);

    bool device_ready(void) const;

    static const struct AP_Param::GroupInfo var_info[];

  private:

    // parameters
    AP_Int8 _type;
    AP_Int8 _control;

    // external references
    AP_MotorController_Backend *_driver;
    AP_SerialManager *_serial_manager;
};
