#!/usr/bin/env python

from __future__ import print_function
import random

from arducopter import *
from pymavlink import *
from pymavlink.mavutil import location
from patrol_manager import *

HOME = mavutil.location(-35.363261, 149.165230, 584, 353)
MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE = ((1 << 0) | (1 << 1) | (1 << 2))
MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE = ((1 << 3) | (1 << 4) | (1 << 5))
MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE = ((1 << 6) | (1 << 7) | (1 << 8))
MAVLINK_SET_POS_TYPE_MASK_FORCE = (1 << 9)
MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE = (1 << 10)
MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE = (1 << 11)
MAV_FRAMES = {"MAV_FRAME_GLOBAL": mavutil.mavlink.MAV_FRAME_GLOBAL,
              "MAV_FRAME_GLOBAL_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
              "MAV_FRAME_GLOBAL_RELATIVE_ALT": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
              "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
              "MAV_FRAME_GLOBAL_TERRAIN_ALT": mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
              "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT}

battery_capacity = 18000

#Metric :
# Number of completed patrol cycle
# calcule avg_idleness[i], stddev_idleness[i], number_of_visits sur les points à visité
# 	fprintf(file,"\nNode idleness\n");
# 	fprintf(file,"   worst_avg_idleness (graph) = %.1f\n", worst_avg_idleness);
# 	fprintf(file,"   avg_idleness (graph) = %.1f\n", avg_graph_idl);
# 	fprintf(file,"   median_idleness (graph) = %.1f\n", median_graph_idl);
# 	fprintf(file,"   stddev_idleness (graph) = %.1f\n", stddev_graph_idl);
#	fprintf(file,"\nGlobal idleness\n");
# fprintf(file,"   min = %.1f\n", min_idleness);
# fprintf(file,"   avg = %.1f\n", gavg);
# fprintf(file,"   stddev = %.1f\n", gstddev);
# fprintf(file,"   max = %.1f\n", max_idleness);

# distance interdrone
# robot perdu

class PartrolSimCopter(AutoTestCopter):
    def __init__(self, *args, **kwargs):
        super(PartrolSimCopter, self).__init__(*args, **kwargs)
        self.wp_accuracy = None

    def set_position_global_int(self, latitude, longitude, altitude, has_alt, timeout=100):
        """set position message in guided mode."""

        targetpos = self.mav.location()

        def send_target_position(lat, lng, alt, mav_frame):
            self.mav.mav.set_position_target_global_int_send(
                0,  # timestamp
                1,  # target system_id
                1,  # target component id
                mav_frame,
                MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE |
                MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE |
                MAVLINK_SET_POS_TYPE_MASK_FORCE |
                MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE |
                MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE,
                lat * 1.0e7,  # lat
                lng * 1.0e7,  # lon
                alt,  # alt
                0,  # vx
                0,  # vy
                0,  # vz
                0,  # afx
                0,  # afy
                0,  # afz
                0,  # yaw
                0,  # yawrate
            )

        targetpos.lat = latitude
        targetpos.lng = longitude
        targetpos.alt = altitude
        send_target_position(targetpos.lat, targetpos.lng, targetpos.alt, mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
        if not self.wait_location(targetpos, accuracy=self.wp_accuracy, timeout=timeout,
                                  target_altitude=(targetpos.alt if has_alt else None),
                                  height_accuracy=2):
            raise NotAchievedException()


    def testSKT(self):
        """Autotest ArduCopter in SITL."""
        if not self.hasInit:
            self.init()

        self.fail_list = []

        timeout = 20*60
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.mav.wait_heartbeat()
            self.wait_mode('STABILIZE')
            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()
            tstart = self.get_sim_time()
            self.set_parameter("BATT_CAPACITY", battery_capacity)
            self.set_parameter("FS_GCS_ENABLE", 0)
            self.set_throttle_zero()
            self.mavproxy.send('mode guided\n')
            self.wait_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()

            if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                     mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                     mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                     mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                     mavutil.mavlink.MAV_TYPE_COAXIAL,
                                     mavutil.mavlink.MAV_TYPE_TRICOPTER,
                                     mavutil.mavlink.MAV_TYPE_SUBMARINE]:
                self.user_takeoff(alt_min=20)

            if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                     mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                     mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                     mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                     mavutil.mavlink.MAV_TYPE_COAXIAL,
                                     mavutil.mavlink.MAV_TYPE_TRICOPTER,
                                     mavutil.mavlink.MAV_TYPE_SUBMARINE]:
                self.wp_accuracy = self.get_parameter("WPNAV_RADIUS", retry=2)
                self.wp_accuracy = self.wp_accuracy * 0.01  # cm to m
            if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_FIXED_WING,
                                     mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                                     mavutil.mavlink.MAV_TYPE_SURFACE_BOAT]:
                self.wp_accuracy = self.get_parameter("WP_RADIUS", retry=2)
            if self.wp_accuracy is None:
                raise ValueError()
            while self.get_sim_time() < tstart + timeout:
                target = random.randint(0, len(patrol_list)-1)
                self.set_position_global_int(patrol_list[target].lat, patrol_list[target].lng, patrol_list[target].alt, True)

        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            self.fail_list.append("Failed with timeout")
        self.close()

        if len(self.fail_list):
            self.progress("FAILED : %s" % self.fail_list)
            return False
        return True

binary = util.reltopdir(os.path.join('build', "sitl", 'bin', "arducopter"))


fly_opts = {
    "binary": binary,
    "frame": "quad",
    "viewerip": "127.0.0.1",
    "use_map": True,
    "valgrind": False,
    "gdb": False,
    "gdbserver": False,
    "speedup": 5,
}

sktcopter = PartrolSimCopter(**fly_opts)
sktcopter.testSKT()
