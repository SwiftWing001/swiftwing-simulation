#!/usr/bin/env python3
import numpy as np
from math import acos
import time
import rospy
import sys
from rospkg import RosPack
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
a = RosPack()
path_now = a.get_path("multi_demo") + "/scripts"
sys.path.append(path_now)
sys.path.append(path_now + "/swarm_lf")
from swarm_lf.PathFollow import PathFollow
from swarm_lf.DesirePath import DesirePath
path_now = a.get_path("uav_controller") + "/scripts/uav_model_lib"
sys.path.append(path_now)
from spawn_uav import uav

class LeaderUAV(PathFollow):
    def __init__(self, start_pos, desire_path):
        super().__init__(start_pos, desire_path)
        self.desire_p_pub = rospy.Publisher("/leader/desire_p", Vector3, queue_size=1)
        self.desire_v_pub = rospy.Publisher("/leader/desire_v", Vector3, queue_size=1)
        self.desire_roll_pub = rospy.Publisher("/leader/desire_roll", Float64, queue_size=1)
        # self.uav_handle = uav_handle
        self.time_start = time.time()

    def OriCal(self):
        # (二维)输入期望位置和期望速度矢量，输出期望偏航角、滚转角和轴向加速度
        u = self.Acal()
        a, phi = self.AngleCal(u)
        _, _, desire_v, _ = self.path.NearestP(self.pose[[0, 1]])
        psi = np.arctan2(desire_v[1], desire_v[0])
        self.desire_roll_pub.publish(Float64(phi))
        return psi, -phi, a
    
    def PitchCal(self, d_h):
        _, _, desire_v, _ = self.path.NearestP(self.pose[[0, 1]])
        pitch = np.arctan2(d_h, np.linalg.norm(desire_v))
        return pitch

    def StatusUpdate(self, self_p, self_v):
        self.pose = np.copy(self_p)
        self.vel = np.copy(self_v)
        self.status = np.array([self.pose[0], self.pose[1], self.vel[0], self.vel[1]])
        min_p, _, desire_v, _ = self.path.NearestP(self.status[[0, 1]])

        p_vec = Vector3(self.pose[0], self.pose[1], self.pose[2])
        self.desire_p_pub.publish(p_vec)

        v_vec = Vector3(self.vel[0], self.vel[1], self.vel[2])
        self.desire_v_pub.publish(v_vec)

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
    
    rospy.init_node("leader_uav" + str(uav_id))
    
    # uav_handle实例化
    uav_handle = uav(uav_amount=uav_amount)

    # 领航者实例化
    points = np.array([[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.], [0., 0.]]) # 定义飞行轨迹
    desire_v = 18. # 定义巡航速度
    path = DesirePath(points, desire_v)
    path_follow = LeaderUAV(np.array([0., 0.]), path)

    fly_height = 80.
    while not rospy.is_shutdown():
        path_follow.StatusUpdate(uav_handle.gazebo_pose, uav_handle.global_vel)
        yaw, roll, a = path_follow.OriCal()
        desire_d_h = (fly_height - path_follow.pose[2]) * 0.4
        pitch = np.arcsin(desire_d_h / (uav_handle.global_vel[0]**2 + uav_handle.global_vel[1]**2 + desire_d_h**2)**0.5)
        uav_handle.control_send([roll, pitch, yaw, a], "att")
        time.sleep(0.1)
