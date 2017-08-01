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
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    // get the absolute WGS-84 position
    struct Location _abspos;
    _ahrs_ekf.get_position(_abspos);
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    // get the absolute WGS-84 position
    struct Location _abspos;
    _ahrs_ekf.get_position(_abspos);
    return _abspos.lng;
}



/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    // get the velocity relative to the local earth frame
    Vector3f velNED;
    Vector3f _velocity_cm; // NEU
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100.0f; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }
    return norm(_velocity_cm.x, _velocity_cm.y);
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
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    // get the NED position relative to the local earth frame origin
    Vector3f posNED;
    Vector3f _relpos_cm;   // NEU
    if (_ahrs_ekf.get_relative_position_NED_origin(posNED)) {
        _relpos_cm.x = posNED.x * 100.0f; // convert from m to cm
        _relpos_cm.y = posNED.y * 100.0f; // convert from m to cm
        _relpos_cm.z = - posNED.z * 100.0f; // convert from m in NED to cm in NEU
    }
    return _relpos_cm.z;
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

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    Vector3f velNED;
    Vector3f _velocity_cm; // NEU
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100.0f; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }
    return _velocity_cm.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
