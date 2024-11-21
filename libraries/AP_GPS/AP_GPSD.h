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

//
//  GPSD GPS driver which accepts gps position data from a GPSD deamon
//  Initial code by Pierre Kancir
//

#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_GPSD_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"

#include <AP_HAL/AP_HAL.h>

#include <libgpsmm.h>

class AP_GPS_GPSD : public AP_GPS_Backend
{
public:
    using AP_GPS_Backend::AP_GPS_Backend;

    // Methods
    bool read() override;

    // driver specific health, returns true if the driver is healthy
    bool is_healthy(void) const override;

    bool is_configured(void) const override {
        return true;
    }

private:
    gpsmm gps_rec;
    struct gps_data_t* newdata;

#endif //AP_GPS_GPSD_ENABLED
