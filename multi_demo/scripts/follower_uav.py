#!/usr/bin/env python3
import numpy as np
from math import acos
import time
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import sys
from rospkg import RosPack
a = RosPack()
path_now = a.get_path("multi_demo") + "/scripts"
sys.path.append(path_now)
sys.path.append(path_now + "/swarm_lf")
from swarm_lf.Follower import Follower
from swarm_lf.FormationFun import *
path_now = a.get_path("uav_controller") + "/scripts/uav_model_lib"
sys.path.append(path_now)
from spawn_uav import uav


class FollowUAV(Follower):
    def __init__(self, start_pos, offset):
        super().__init__(start_pos, offset)
        
        self.leader_v_sub = rospy.Subscriber("/leader/desire_v", Vector3, self.LeaderVCB, queue_size=2)
        self.leader_p_sub = rospy.Subscriber("/leader/desire_p", Vector3, self.LeaderPCB, queue_size=2)
        self.leader_roll_sub = rospy.Subscriber("/leader/desire_roll", Float64, self.LeaderRollCB, queue_size=2)

        self.leader_p = np.zeros([3])
        self.leader_v = np.zeros([3])
        self.leader_roll = 0.

        self.pose = np.zeros([3])
        self.vel = np.zeros([3])

        self.leader_p_sub_time = time.time()

    def LeaderPCB(self, msg):
        self.leader_p = np.array([msg.x, msg.y, msg.z])
        self.leader_p_sub_time = time.time()

    def LeaderVCB(self, msg):
        self.leader_v = np.array([msg.x, msg.y, msg.z])

    def LeaderRollCB(self, msg):
        self.leader_roll = msg.data

    def Acal(self):
        leader_p = np.array([self.leader_p[0], self.leader_p[1]])
        leader_v = np.array([self.leader_v[0], self.leader_v[1]])
        leader_r = leader_v @ leader_v / 9.8 / np.tan(self.leader_roll)
        return super().Acal(leader_p, leader_v, leader_r)
    
    def OriCal(self):
        # (二维)输入期望位置和期望速度矢量，输出期望偏航角、滚转角和加速度
        u = self.Acal()
        a, phi = self.AngleCal(u)
        leader_v = np.array([self.leader_v[0], self.leader_v[1]])
        desire_v_unit = leader_v / np.linalg.norm(leader_v)
        psi = np.arctan2(self.leader_v[1], self.leader_v[0])
        return psi, -phi, a
    
    def PitchCal(self, d_h):
        desire_v = self.leader_v[[0, 1]]
        pitch = np.arctan2(d_h, np.linalg.norm(desire_v))
        return pitch

    def StatusUpdate(self, self_p, self_v):
        self.pose = np.copy(self_p)
        self.vel = np.copy(self_v)
        self.status = np.array([self.pose[0], self.pose[1], self.vel[0], self.vel[1]])
        

if __name__ == "__main__":
    time.sleep(2)
    # ros 初始化
    try:
        uav_id = int(rospy.get_param("uav_id"))
        print("====Start with 'uav_id'= " + str(uav_id))
    except KeyError:
        uav_id = 1
        rospy.logwarn("Start without 'uav_id', set uav_id = " + str(uav_id))
    
    try:
        uav_amount = int(rospy.get_param("uav_amount"))
        print("====Start with 'uav_amount'= " + str(uav_amount))
    except KeyError:
        uav_amount = 2
        rospy.logwarn("Start without 'uav_amount', set uav_amount=" + str(uav_amount))

    rospy.init_node("follower_" + str(uav_id))
    
    form_distance = 20.
    offset_list = CircleForm(uav_amount - 1, form_distance)
    offset = offset_list[uav_id - 1, :]
    follow_uav = FollowUAV(np.array([0., 0.]), offset)
    print("offset_list: ", offset_list)
    
    # uav_handle实例化
    uav_handle = uav(uav_amount=uav_amount, uav_index = uav_id)

    formation_threshold = 80  # start fly as plane
    fly_status = 0  # =0: go higher as drone; =1: drone2plane; =2: do task as plane
    time_mode_change = time.time()
    time_stop = time.time()

    t_before = time.time()
    while not rospy.is_shutdown():
        
        t_gap = time.time() - t_before
        t_before = time.time()

        follow_uav.StatusUpdate(uav_handle.gazebo_pose, uav_handle.global_vel)
        desire_d_h = (formation_threshold - follow_uav.pose[2]) * 0.4
        yaw, roll, a = follow_uav.OriCal()
            
        pitch = np.arcsin(desire_d_h / (uav_handle.global_vel[0]**2 + uav_handle.global_vel[1]**2 + desire_d_h**2)**0.5)
        uav_handle.control_send([roll, pitch, yaw, a], "att")
        time.sleep(0.05)

