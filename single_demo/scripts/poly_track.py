#!/usr/bin/env python3
import rospy
import numpy as np
import time
import os
import sys

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_gvflib = dir_mytest + "/scripts/gvf_lib"
sys.path.insert(0, dir_gvflib)

from gvf_lib.PolynomialTraj import PolynomialTraj3Order
from gvf_lib.gvf_poly import GVF
from uav_lib.spawn_uav import uav

# create a gvf instance
points = np.array([[0., 0., 150.],
                    [400., 0., 150.],
                    [400., 1000., 150.],
                    [800., 1000., 150.],
                    [800., 0., 150.],
                    [1200., 0., 150.],
                    [1200., 2400., 150.],
                    [800., 2400., 150.],
                    [800., 1400., 150.],
                    [400., 1400., 150.],
                    [400., 2400., 150.],
                    [0., 2400., 150.]]) / 4.
points[:, 2] = 100.

dimension = 3
traj = PolynomialTraj3Order(dimension, points, loop_flag=1)
gvf = GVF(traj, k=0.003, mc_flag=0, ori=1)
t_now = 0.

# create a uav instance
uav_type = "plane"
uav_index = 0
uav_handle = uav(uav_type, uav_index)
cruise_speed = 15.
rospy.init_node("poly_track")

t_before = time.time()
t_now = 0.
while not rospy.is_shutdown():
    # get control time gap
    t_gap = time.time() - t_before
    t_before = time.time()

    # compute velocity  vector_sp
    pose = np.copy(uav_handle.local_pose)
    desire_vec = gvf.GetVec(pose, t_now)
    t_now += t_gap * desire_vec[-1] * cruise_speed
    vector_sp = cruise_speed * desire_vec[0:3]
    
    # send contrl vector
    control_type = "vel"
    uav_handle.control_send(vector_sp, control_type)
    print("control type: ", control_type, "   cmd: ", vector_sp)

    rospy.sleep(0.01)
    
    

