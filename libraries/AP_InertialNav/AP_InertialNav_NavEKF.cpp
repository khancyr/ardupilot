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
