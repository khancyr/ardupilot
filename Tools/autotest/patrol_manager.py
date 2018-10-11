from __future__ import print_function
from pymavlink.mavutil import location
import pymavlink.mavutil as mavutil
import sys
import time

patrol_list = [location(-35.363261, 149.165230, 614, 353),
               location(-35.363361, 149.165230, 614, 353),
               location(-35.363261, 149.165330, 614, 353),]

srcSystem = 1 #int(sys.argv[2])
mav_get = mavutil.mavlink_connection(
    'udpin:0.0.0.0:14550', source_system=srcSystem)
mav_send = mavutil.mavlink_connection(
    'tcp:127.0.0.1:5773', source_system=1)
mav_send2 = mavutil.mavlink_connection(
    'tcp:127.0.0.1:5783', source_system=1)

while (True):
    msg = mav_get.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg.get_srcSystem() == srcSystem:
        #print("Message from %d: %s" % (msg.get_srcSystem(), msg))
        mav_send.mav.global_position_int_send(msg.time_boot_ms, msg.lat, msg.lon, msg.alt, msg.relative_alt, msg.vx, msg.vy, msg.vz, msg.hdg)
        mav_send2.mav.global_position_int_send(msg.time_boot_ms, msg.lat, msg.lon, msg.alt, msg.relative_alt, msg.vx, msg.vy, msg.vz, msg.hdg)