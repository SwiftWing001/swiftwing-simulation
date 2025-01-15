#!/usr/bin/env python3
import rospy
import numpy as np
import time
import os
import sys

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_gvflib = dir_mytest + "/scripts/gvf_lib"
sys.path.insert(0, dir_gvflib)

from gvf_lib.gvf_lissajous import GVF
from uav_lib.spawn_uav import uav

# create a gvf instance
gvf = GVF()
gvf.k = np.array([[0.005,0,0], [0,0.01,0], [0,0,0.2]])

# create a uav instance
uav_type = "plane"
uav_index = 0
uav_handle = uav(uav_type, uav_index)
cruise_speed = 15.

rospy.init_node("lissajous_track")

t_before = time.time()
w = 0.
while not rospy.is_shutdown():
    # get control time gap
    t_gap = time.time() - t_before
    t_before = time.time()

    # compute velocity  vector_sp
    pose = np.copy(uav_handle.local_pose)
    pose = np.append(pose, w)
    guide_vec = gvf.GetGuidVec(pose)
    w_v = guide_vec[-1]
    w += cruise_speed*w_v * t_gap
    vector_sp = cruise_speed * guide_vec[0:3]

    # send contrl vector
    control_type = "vel"
    uav_handle.control_send(vector_sp, control_type)
    print("control type: ", control_type, "   cmd: ", vector_sp)

    rospy.sleep(0.01)
