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

from uav_model_lib.FW_class import VectorControlPlane


file_name = os.path.basename(__file__)
print(file_name) 
rospy.init_node(file_name)
plane = VectorControlPlane()
plane.psi_pid.kd = 2.5
plane.psi_pid.ki = 0.1
plane.psi_pid.kd = 0.0

plane.pitch_h_pid.kp = 0.2
plane.pitch_h_pid.ki = 0.0
plane.pitch_h_pid.kd = 0.0

t_before = time.time()
control_vector = Vector3()
rospy.Subscriber

while not rospy.is_shutdown():
    rospy.loginfo(plane.state.mode)
    if plane.state.armed == False:
        plane.takeoff()
    
    # Get control time gap
    t_gap = time.time() - t_before
    t_before = time.time()
    
    if plane.control_mode == "vel":
        # Generate the velocity vector setpoint
        v_sp = np.copy(plane.velocity_sp)

        # Generate attitude setpoint according to the velocity vector setpoint
        dot_h_sp, yaw_sp, roll_sp, desire_a = plane.Vec2Ori(v_sp, dt=t_gap, method=1)
        force, pitch_sp = plane.tecsControl(desire_a, dot_h_sp, roll_sp, t_gap)
        print('roll_sp, -pitch_sp, yaw_sp',roll_sp, -pitch_sp, yaw_sp)
        plane.MotionControl("att", [roll_sp, -pitch_sp, yaw_sp], control_force=force)
    elif plane.control_mode == "att":
        roll_sp = plane.attitude_sp[0]
        pitch_sp = plane.attitude_sp[1]
        yaw_sp = plane.attitude_sp[2]
        force = plane.attitude_sp[3]

        print('roll_sp, -pitch_sp, yaw_sp',roll_sp, -pitch_sp, yaw_sp)
        plane.MotionControl("att", [roll_sp, -pitch_sp, yaw_sp], control_force=force)

    # if plane.state.mode == "AUTO.LOITER":
    #     print("loitering---")
    #     plane.set_offboard()
    #     rospy.loginfo(plane.state.mode)

    if plane.state.mode != "OFFBOARD":
        plane.set_offboard()

    if plane.state.mode == "OFFBOARD":
        print('OFFBOARD flying')

    rospy.sleep(0.01)
    

