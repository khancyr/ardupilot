#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    // pull position
    Location loc;
    ahrs.get_position(loc);
    current_loc.lng = loc.lng;
    current_loc.lat = loc.lat;

    // exit immediately if we do not have an altitude estimate
    nav_filter_status filt_status;
    ahrs.get_filter_status(filt_status);
    if (!filt_status.flags.vert_pos) {
        return;
    }

    // without home return alt above the EKF origin
    if (ap.home_state == HOME_UNSET) {
        // with inertial nav we can update the altitude and climb rate at 50hz
        // get the D position relative to the local earth frame origin
        float posD;
        if (ahrs.get_relative_position_D_origin(posD)) {
            current_loc.alt = -posD * 100;  // convert from m in NED to cm in NEU
        }
    } else {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = pv_alt_above_home(ahrs.get_altitude());
    }

    // set flags and get velocity
    current_loc.flags.relative_alt = true;
    Vector3f vel;
    ahrs.get_velocity_NEU_cm(vel);
    climb_rate = vel.z;
}
