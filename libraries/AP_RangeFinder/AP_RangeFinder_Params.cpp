#include "AP_RangeFinder_Params.h"
#include "AP_RangeFinder.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_RangeFinder_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini
    // @User: Standard
    AP_GROUPINFO("TYPE",    1, AP_RangeFinder_Params, type, 0),

    // @Param: MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
    // @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_CM",  6, AP_RangeFinder_Params, min_distance_cm, 20),

    // @Param: MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
    // @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_CM",  7, AP_RangeFinder_Params, max_distance_cm, 700),

    // @Param: GNDCLEAR
    // @DisplayName: Distance (in cm) from the range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 5 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GNDCLEAR", 12, AP_RangeFinder_Params, ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADDR", 23, AP_RangeFinder_Params, address, 0),

    // @Param: POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the first rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the first rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the first rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS", 49, AP_RangeFinder_Params, pos_offset, 0.0f),

    // @Param: ORIENT
    // @DisplayName: Rangefinder orientation
    // @Description: Orientation of rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 53, AP_RangeFinder_Params, orientation, ROTATION_PITCH_270),

    AP_GROUPEND
};

AP_RangeFinder_Params::AP_RangeFinder_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
