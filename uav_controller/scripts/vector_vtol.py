#!/usr/bin/env python3
import numpy as np
from math import acos
import time
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import sys
import os

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/scripts/uav_model_lib"
sys.path.insert(0, dir_mylib)
print(dir_mylib)

from uav_model_lib.VTOL_class import VectorControlVtol

global control_mode
control_mode = "none"

desire_vec = np.zeros([3])
def DesireVelCB(msg):
    desire_vec[0] = msg.x
    desire_vec[1] = msg.y
    desire_vec[2] = msg.z
    global control_mode
    control_mode = "vel"
    print("listen: ", control_mode)

desire_att = np.zeros([4])
def DesireAttCB(msg):
    desire_att[0] = msg.data[0]
    desire_att[1] = msg.data[1]
    desire_att[2] = msg.data[2]
    desire_att[3] = msg.data[3]
    global control_mode
    control_mode = "att"

if __name__ == "__main__":
    try:
        uav_id = int(rospy.get_param("uav_id"))
        print("====Start with 'uav_id'= " + str(uav_id))
    except KeyError:
        rospy.logwarn("Start without 'uav_id', set uav_id = 0")
        uav_id = 0
    try:
        uav_amount = int(rospy.get_param("uav_amount"))
        print("====Start with 'uav_amount'= " + str(uav_amount))
    except KeyError:
        rospy.logwarn("Start without 'uav_amount', set uav_amount = 2")
        uav_amount = 2

    rospy.init_node("control_uav" + str(uav_id))

    # subscribe desire vector
    if uav_amount == 1:
        topic_form = ""
    else:
        topic_form = "/uav" + str(uav_id)
    desire_vec_sub = rospy.Subscriber(topic_form + "/control_signal/vector", Vector3, DesireVelCB)
    desire_att_sub = rospy.Subscriber(topic_form + "/control_signal/att", Float64MultiArray, DesireAttCB)

    # create control uav class instance
    control_uav = VectorControlVtol(uav_id, uav_amount)

    formation_threshold = 80  # start fly as plane
    fly_status = 0  # =0: go higher as drone; =1: drone2plane; =2: do task as plane
    time_mode_change = time.time()
    time_stop = time.time()

    t_before = time.time()
    while not rospy.is_shutdown():
        print("control_mode: ", control_mode)
        
        t_gap = time.time() - t_before
        t_before = time.time()

        if not control_uav.armed:
            rospy.loginfo("uav " + str(uav_id) + " set arm")
            control_uav.Arming(True)
        elif not control_uav.Manual() == "success":
            rospy.loginfo("uav " + str(uav_id) + " set offboard")

        if fly_status == 0:
            desire_pose = [0, 0, formation_threshold + 20]
            control_uav.MotionControl("pos", desire_pose)
            time_mode_change = time.time()
            print("uav_pos: ", control_uav.pose)
            
            now_pose = control_uav.pose[:]
            if control_uav.pose[2] > formation_threshold:
                fly_status = 1
                control_uav.MotionControl("pos", now_pose)
            time_mode_change = time.time()

        elif fly_status == 1:   # time.time() - time_mode_change < 8:  # stayble
            fly_status = 1
            if control_uav.fly_mode == "drone":
                rospy.loginfo("uav " + str(uav_id) + " change fly mode")
                trans_flag = control_uav.FlyModeTrans("plane")
                time_mode_change = time.time()

            if control_uav.fly_mode == "plane":
                fly_status = 2
                rospy.loginfo("uav " + str(uav_id) + " is plane now")
            elif control_uav.fly_mode == "drone2plane":
                time_mode_change = time.time()
                rospy.loginfo("uav " + str(uav_id) + " mode changing")
            else:
                time_mode_change = time.time()
                rospy.loginfo("uav " + str(uav_id) + " mode doesn't change")

            desire_pose = control_uav.pose[:]
            desire_pose[0] = control_uav.pose[0] + control_uav.avoid[0] + 1000
            desire_pose[1] = control_uav.pose[1] + control_uav.avoid[1] 
            desire_pose[2] = formation_threshold + control_uav.avoid[2]
            
            control_uav.MotionControl("pos", desire_pose, 0.8)
    
            if time.time() - time_mode_change > 8:
                fly_status = 2
        else:
            pos_now = np.array([control_uav.pose])
            pos_now = np.squeeze(pos_now)
            

            if control_mode == "vel":
                desire_h_d, desire_psi, desire_phi, desire_a = control_uav.Vec2Ori(desire_vec, dt=t_gap, method=1)
                force, desire_pitch = control_uav.tecsControl(desire_a, desire_vec, desire_phi, t_gap)
                print("force: ", force)
                control_uav.MotionControl("att", [desire_phi, -desire_pitch, desire_psi], force)
            elif control_mode == "att":
                desire_roll = desire_att[0]
                desire_pitch = desire_att[1]
                desire_yaw = desire_att[2]
                a = desire_att[3]
                desire_d_h = np.linalg.norm(control_uav.velocity) * np.sin(desire_pitch)
                desire_v = np.copy(control_uav.velocity)
                desire_v[2] = desire_d_h

                force, desire_pitch = control_uav.tecsControl(a, desire_v, desire_roll, dt=t_gap)

                print("force: ", force)
                control_uav.MotionControl("att", [desire_roll, -desire_pitch, desire_yaw], force)
            

        time.sleep(0.01)
