#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    // pull position
    if (!ahrs.get_origin(ekf_origin)) {
        return;
    }
    Location loc{};
    if (!ahrs.get_position(loc)) {
        return;
    }
    current_loc.lng = loc.lng;
    current_loc.lat = loc.lat;

    // exit immediately if we do not have an altitude estimate
    nav_filter_status filt_status;
    if (!ahrs.get_filter_status(filt_status)) {
        return;
    }
    if (!filt_status.flags.vert_pos) {
        return;
    }

    // without home return alt above the EKF origin
    if (!ahrs.home_is_set()) {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = inertial_nav.get_altitude();
    } else {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = pv_alt_above_home(inertial_nav.get_altitude());
    }

    // set flags and get velocity
    current_loc.flags.relative_alt = true;
    climb_rate = inertial_nav.get_velocity_z();
}
