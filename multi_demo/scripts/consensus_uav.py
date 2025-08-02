#!/usr/bin/env python3
import numpy as np
from math import acos
import time
import rospy
import sys
from rospkg import RosPack
from geometry_msgs.msg import Vector3

a = RosPack()
path_now = a.get_path("multi_demo") + "/scripts"
sys.path.append(path_now)
sys.path.append(path_now + "/swarm_lf")
from swarm_lf.ConsensusFollow import ConsensusFollow
from swarm_lf.DesirePath import DesirePath
from swarm_lf.FormationFun import *
path_now = a.get_path("uav_controller") + "/scripts/uav_model_lib"
sys.path.append(path_now)
from spawn_uav import uav

class ConsensusUAV(ConsensusFollow):
    def __init__(self, start_pos, desire_path, L, self_id, offset):
        super().__init__(start_pos, desire_path, L, self_id, offset)
        # 发送话题
        topic_name = "/uav" + str(int(self_id)) + "/consensus_data"
        self.consensus_data_pub = rospy.Publisher(topic_name, Vector3, queue_size=1)
        # 接收话题
        self.subs = []
        self.uav_amount = np.shape(self.L)[0]
        for i in range(self.uav_amount):
            if int(i) == int(self.id):
                continue
            elif not self.L[self.id, i] == 0:
                print(i)
                sub_name = "/uav" + str(int(i)) + "/consensus_data"
                self.subs.append(rospy.Subscriber(sub_name, Vector3, self.consensus_cb, (int(i)), queue_size = 2))
        # 邻居信息储存
        self.neighbor_msg = np.zeros([self.uav_amount, 2])
        self.time_start = time.time()

    def consensus_cb(self, msg, argvs):
        index = int(argvs)
        self.neighbor_msg[index, 0] = msg.x
        self.neighbor_msg[index, 1] = msg.y
        # print(" uav ", self.id, " callback: ", index)

    def OriCal(self):
        # (二维)输入期望位置和期望速度矢量，输出期望偏航角、滚转角和轴向加速度
        u = self.Acal(self.neighbor_msg)
        a, phi = self.AngleCal(u)
        _, _, desire_v, _ = self.path.NearestP(self.pose[[0, 1]])
        psi = np.arctan2(desire_v[1], desire_v[0])
        return psi, -phi, a
    
    def PitchCal(self, d_h):
        _, _, desire_v, _ = self.path.NearestP(self.pose[[0, 1]])
        pitch = np.arctan2(d_h, np.linalg.norm(desire_v))
        return pitch

    def StatusUpdate(self, self_p, self_v):
    	# 数据更新
        self.pose = np.copy(self_p)
        self.vel = np.copy(self_v)
        self.status = np.array([self.pose[0], self.pose[1], self.vel[0], self.vel[1]])
        # 参考点更新
        p_now = self.status[[0, 1]]
        v_now = self.status[[2, 3]]
        ref_psi_sin = v_now[1] / np.linalg.norm(v_now)
        ref_psi_cos = v_now[0] / np.linalg.norm(v_now)
        trans_m = np.array([[ref_psi_cos, -ref_psi_sin],
                            [ref_psi_sin, ref_psi_cos]])
        ref_p = np.reshape(p_now, [2]) - trans_m @ self.offset
        self.ref_status[[0, 1]] = ref_p
        self.ref_status[[2, 3]] = v_now
        self.neighbor_msg[self.id, :] = ref_p
        
        p_vec = Vector3(self.ref_status[0], self.ref_status[1], self.pose[2])
        self.consensus_data_pub.publish(p_vec)

def CircleCommunicationMGenerate(uav_amount):
    M = np.eye(uav_amount) * 2.
    for i in range(uav_amount):
        if i == 0:
            M[i, 1] = -1
            M[i, -1] = -1
        elif i == uav_amount-1:
            M[i, 0] = -1
            M[i, -2] = -1
        else:
            M[i, i-1] = -1
            M[i, i+1] = -1
    return M

if __name__ == "__main__":
    # ros 初始化
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
        uav_amount = 2
        rospy.logwarn("Start without 'uav_amount', set uav_amount = " + str(uav_amount))
    
    rospy.init_node("consensus_uav" + str(uav_id))
    
    # uav_handle实例化
    uav_handle = uav(uav_amount=uav_amount, uav_index=uav_id)

    # 轨迹实例化
    points = np.array([[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.], [0., 0.]]) # 定义飞行轨迹
    desire_v = 20. # 定义巡航速度
    path = DesirePath(points, desire_v)

    # 定义编队
    form_distance = 20.
    formation = CircleForm(uav_amount, form_distance)

    # 定义通信拓扑
    L = CircleCommunicationMGenerate(uav_amount)


    consensus_uav = ConsensusUAV(np.array([0., 0.]), path, L, uav_id, formation[uav_id, :])

    fly_height = 80. - uav_id
    while not rospy.is_shutdown():
        consensus_uav.StatusUpdate(uav_handle.gazebo_pose, uav_handle.global_vel)
        yaw, roll, a = consensus_uav.OriCal()
        desire_d_h = (fly_height - consensus_uav.pose[2]) * 0.4
        if abs(desire_d_h) > 1.:
            desire_d_h = np.sign(desire_d_h) * 1.
        pitch = np.arcsin(desire_d_h / (uav_handle.global_vel[0]**2 + uav_handle.global_vel[1]**2 + desire_d_h**2)**0.5)
        uav_handle.control_send([roll, pitch, yaw, a], "att")
        time.sleep(0.1)
