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
#include "stdint.h"
#include "SIM_I2CDevice.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

namespace SITL {

class Precland_IrlockI2C : public I2CDevice {
public:

    void update(const class Aircraft &aircraft) override {

    }

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return 0;
    }
private:

};

}
