#include "Sub.h"

// read_inertia - read inertia in from accelerometers
void Sub::read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    // pull position from interial nav library
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

    current_loc.alt = inertial_nav.get_altitude();

    // get velocity, altitude is always absolute frame, referenced from
    // water's surface
    Vector3f vel;
    ahrs.get_velocity_NEU_cm(vel);
    climb_rate = vel.z;
}
