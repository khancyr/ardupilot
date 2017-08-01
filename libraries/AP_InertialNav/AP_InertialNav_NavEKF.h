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



private:




    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
};
