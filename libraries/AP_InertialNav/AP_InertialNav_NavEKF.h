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


private:




    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
};
