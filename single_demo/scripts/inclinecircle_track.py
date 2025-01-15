#!/usr/bin/env python3
import rospy
import numpy as np
import os
import sys

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_gvflib = dir_mytest + "/scripts/gvf_lib"
sys.path.insert(0, dir_gvflib)

from gvf_lib.gvf_inclined_circle import InclinePlane, Cylinder, GVF
from uav_lib.spawn_uav import uav


# create a gvf instance
p = InclinePlane(normal_vector=[1.,0.,10.], point=[0.,0.,150.],k=[0.0005,0.01,0.01])
c = Cylinder(center=[0., 0., 0.], r=100., ori=2, k=[0.1,0.1,0.0])
gvf = GVF(surfaces=[c,p])

# create a uav instance
uav_type = "plane"
uav_index = 0
uav_handle = uav(uav_type, uav_index)
cruise_speed = 15.

rospy.init_node("inclined_circle_track")
while not rospy.is_shutdown():
    # compute velocity  vector_sp
    pose = np.copy(uav_handle.local_pose)
    guide_vec = gvf.GetGuideV(pos=pose)
    vector_sp = cruise_speed*guide_vec

    # send contrl vector
    control_type = "vel"
    uav_handle.control_send(vector_sp, control_type)
    print("control type: ", control_type, "   cmd: ", vector_sp)

    rospy.sleep(0.01)
    

