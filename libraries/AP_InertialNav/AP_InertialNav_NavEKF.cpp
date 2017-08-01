#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{

}

/**
 * get_pos_z_derivative - returns the derivative of the z position in cm/s
*/
float AP_InertialNav_NavEKF::get_pos_z_derivative() const
{
    // Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    float _pos_z_rate;
    if (_ahrs_ekf.get_vert_pos_rate(_pos_z_rate)) {
        _pos_z_rate *= 100.0f; // convert to cm/s
        _pos_z_rate = - _pos_z_rate; // InertialNav is NEU
    }
    return _pos_z_rate;
}


/**
 * getHgtAboveGnd - get latest height above ground level estimate in cm and a validity flag
 *
 * @return
 */
bool AP_InertialNav_NavEKF::get_hagl(float &height) const
{
    // true when estimate is valid
    bool valid = _ahrs_ekf.get_hagl(height);
    // convert height from m to cm
    height *= 100.0f;
    return valid;
}

/**
 * get_hgt_ctrl_limit - get maximum height to be observed by the control loops in cm and a validity flag
 * this is used to limit height during optical flow navigation
 * it will return invalid when no limiting is required
 * @return
 */
bool AP_InertialNav_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    // true when estimate is valid
    if (_ahrs_ekf.get_hgt_ctrl_limit(limit)) {
        // convert height from m to cm
        limit *= 100.0f;
        return true;
    }
    return false;
}


#endif // AP_AHRS_NAVEKF_AVAILABLE
