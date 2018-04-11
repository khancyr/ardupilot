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

#include "AP_MotorController.h"
#include "AP_MotorController_Backend.h"

#include "AP_MotorController_RoboClaw.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_MotorController::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Motor Controller device type
    // @Description: What type of Motor Controller is connected
    // @Values: 0:None,1:RoboClaw
    // @User: Advanced
    AP_GROUPINFO("_TYPE", 0, AP_MotorController, _type, 0),

    // @Param: _CONTROL
    // @DisplayName: Motor Controller control type
    // @Description: What type of control the motor controller use
    // @Values: 0:PWM,1:Speed control,2:Position control
    // @User: Advanced
    AP_GROUPINFO("_CONTROL", 0, AP_MotorController, _control, 0),

    AP_GROUPEND
};

AP_MotorController::AP_MotorController()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_MotorController class
void AP_MotorController::init(void) {

    _serial_manager = AP_SerialManager::get_instance();
    if (_driver != nullptr) {
        // init called a 2nd time?
        return;
    }

    // create backend
    switch (_type) {
        case MotorController_TYPE_ROBOCLAW: {
            _driver = new AP_MotorController_RoboClaw(*this, _serial_manager);
            break;
        }
        case MotorController_TYPE_NONE:
        default:break;
    }
}

// update state. This should be called often from the main loop
void AP_MotorController::update(uint32_t motor1, uint32_t motor2) {
    if (!device_ready()) {
        return;
    }
    _driver->update(motor1, motor2);

}

// check if the device is ready
bool AP_MotorController::device_ready(void) const {
    return ((_driver != nullptr) && (_type != MotorController_TYPE_NONE));
}