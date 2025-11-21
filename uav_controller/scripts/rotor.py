#!/usr/bin/env python3
import rospy
import numpy as np
import time
import os
import sys
from geometry_msgs.msg import Vector3

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/scripts/uav_model_lib"
sys.path.insert(0, dir_mylib)
print(dir_mylib)

from uav_model_lib.Rotor_class import uav


file_name = os.path.basename(__file__)
print(file_name) 
rospy.init_node(file_name)


try:
    uav_id = int(rospy.get_param("uav_id"))
    print("====Start with 'uav_id'= " + str(uav_id))
except KeyError:
    rospy.logwarn("====Start without 'uav_id', set uav_id = None")
    uav_id = None

if __name__ == "__main__":
    rotor = uav(type="uav", uav_index=uav_id)
    while not rospy.is_shutdown():
        
        if rotor.state.armed == False:
            answer = rotor.takeoff(h = 30)
            # print("takeoff answer:", answer)
        
        if rotor.local_pose[2] > 25 and rotor.state.mode != "OFFBOARD":
            mode_answer = rotor.set_mode("OFFBOARD")
            if rotor.control_mode == "vel":
                rotor.MotionControl("vel", rotor.velocity_sp)
        elif rotor.state.mode == "OFFBOARD":
            if rotor.control_mode == "vel":
                rotor.MotionControl("vel", rotor.velocity_sp)
        
        time.sleep(0.01)
