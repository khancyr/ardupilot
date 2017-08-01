/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav(),
        _haveabspos(false),
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(float dt);

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    int32_t     get_latitude() const;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * get_pos_z_derivative - returns the derivative of the z position in cm/s
    */
    float    get_pos_z_derivative() const;

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const;

    /**
     * getHgtAboveGnd - get latest altitude estimate above ground level in centimetres and validity flag
     * @return
     */
    bool       get_hagl(float &hagl) const;

    /**
     * get_hgt_ctrl_limit - get maximum height to be observed by the control loops in cm and a validity flag
     * this is used to limit height during optical flow navigation
     * it will return invalid when no limiting is required
     * @return
     */
    bool       get_hgt_ctrl_limit(float& limit) const;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const;

private:




    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
};
