/*
  SITL handling

  This simulates a rangefinder

 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// TODO Improve that to not use analog
/*
  setup the rangefinder with new input
 */
void SITL_State::_update_rangefinder(float range_value)
{
    float altitude = range_value;
    if (range_value == -1) {
        altitude = _sitl->height_agl;
    }

    // sensor position offset in body frame
    Vector3f relPosSensorBF = _sitl->rngfnd_pos_offset;

    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    if (!relPosSensorBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        rotmat.from_euler(radians(_sitl->state.rollDeg),
                radians(_sitl->state.pitchDeg),
                radians(_sitl->state.yawDeg));
        // rotate the offset into earth frame
        Vector3f relPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= relPosSensorEF.z;
    }

    float voltage = 5.0f;
    if (range_value == -1) {
        if (fabsf(_sitl->state.rollDeg) < 90 &&
                fabsf(_sitl->state.pitchDeg) < 90) {
            // adjust for apparent altitude with roll
            altitude /= cosf(radians(_sitl->state.rollDeg)) * cosf(radians(_sitl->state.pitchDeg));

            altitude += _sitl->sonar_noise * _rand_float();
        }
    }
    // Altitude in in m, scaler in meters/volt
    voltage = altitude / _sitl->sonar_scale;
    voltage = constrain_float(voltage, 0, 5.0f);

    if (!is_zero(_sitl->sonar_glitch) && _sitl->sonar_glitch >= (_rand_float() + 1.0f)/2.0f) {
        voltage = 5.0f;
    }

    sonar_pin_value = 1023 * (voltage / 5.0f);
}

#endif
