#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import os

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_gvflib = dir_mytest + "/scripts/gvf_lib"
sys.path.insert(0, dir_gvflib)
from gvf_lib.gvf_circle import circle 
from uav_lib.spawn_uav import uav

# create a gvf instance
c = circle(r=100, k=0.03)
cruise_speed = 15.

# create a uav instance
uav_type = "plane"
uav_index = 0
uav_handle = uav(uav_type, uav_index)

rospy.init_node("circle_track")

while not rospy.is_shutdown():
    # compute velocity  vector_sp
    pose = np.copy(uav_handle.local_pose)
    vector_sp = cruise_speed * c.get_omega(pose[0: 2])
    v_h = (50. - pose[2]) 
    vector_sp = np.append(vector_sp, v_h)

    # send contrl vector
    control_type = "vel"
    uav_handle.control_send(vector_sp, control_type)
    print("control type: ", control_type, "   cmd: ", vector_sp)

    rospy.sleep(0.01)
    

