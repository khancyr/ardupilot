#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include "AP_RangeFinder_Params.h"

class AP_RangeFinder_analog : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_analog(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect(AP_RangeFinder_Params &_params);

    // update state
    void update(void) override;

    static const struct AP_Param::GroupInfo var_info[];

    int8_t get_pin() { return pin; };

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    // update raw voltage
    void update_voltage(void);
    AP_Int8 pin;
    AP_Int16 settle_time_ms;
    AP_Int8  ratiometric;
    AP_Int8  stop_pin;
    AP_Float scaling;
    AP_Float offset;
    AP_Int8  function;
    AP_HAL::AnalogSource *source;
};
