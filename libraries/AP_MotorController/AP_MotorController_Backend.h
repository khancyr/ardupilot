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
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorController.h"

class AP_MotorController_Backend {
  public:
    // constructor. This incorporates initialisation as well.
    AP_MotorController_Backend(AP_MotorController &frontend);

    // update
    virtual void update(uint32_t motor1, uint32_t motor2) = 0;

    int8_t get_control_type() const {
        return _frontend._control;
    }

  protected:

    // references
    AP_MotorController &_frontend;
};
