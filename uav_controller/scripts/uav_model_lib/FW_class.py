#!/usr/bin/env python
import rospy
import numpy as np
import copy
import time
import os
import sys
import tf

from geographiclib.geodesic import Geodesic
from tf.transformations import euler_from_quaternion, quaternion_matrix
from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
# from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# from rospkg import RosPack
# a = RosPack()
# path_now = a.get_path("vtol_control") + "/script"
# sys.path.append(path_now)
# sys.path.append(path_now + "/uav_model_lib")
dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/script/uav_model_lib"
sys.path.insert(0, dir_mylib)
print(dir_mylib)


# from gvf_circle import circle 
# from gvf_lissajous import Path, GVF
from myPID import PID
from useful_function import Eular2Quater
from math import pi

class uav():
    def __init__(self, type = "plane", uav_index=0):
        self.type = type        # this str will use in publisher/subscriber establish
        self.index = uav_index  # uav's index in group, every uav has a unique one
        self.topic_form = "/" + self.type + str(self.index)
        self.topic_form = ""
        self.armed = False
        self.state = State

        self.mode = "Init"
        self.avoid = [0.0, 0.0, 0.0]

        self.body_acc = [0.1, 0.1, 0.1]
        self.global_acc = [0.1, 0.1, 0.1]
        self.local_pose = [0.1, 0.1, 0.1]
        self.global_pose = [0.1, 0.1, 0.1]
        self.local_vel = [0.1, 0.1, 0.1]
        self.global_vel = [0.1, 0.1, 0.1] 
        self.body_vel = [0.1, 0.1, 0.1] 
        self.orientation = [0.1, 0.1, 0.1, 0.1]
        self.velocity_sp = [0.1, 0.1, 0.1, 0.1]
        self.R_enu2ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.time_now = time.time()  
        self.mavros_subscriber()
        self.mavros_client()
        self.mavros_publisher()

        self.control_mode = "none"

    # def __del__(self):
    #     self.local_pose_sub.unregister()
    #     self.global_pose_sub.unregister()
    #     self.local_vel_sub.unregister()
    #     self.global_vel_sub.unregister()
    #     self.body_vel_sub.unregister()
    #     self.acc_sub.unregister()
    #     self.uav_state_sub.unregister()
    #     self.motivation_pub.unregister()
    #     self.arm_client.close()
    #     self.set_mode_client.close()
    #     rospy.loginfo("uav " + str(self.index) + " delete success")

    def mavros_subscriber(self):
        self.local_pose_sub = rospy.Subscriber(
            self.topic_form + "/mavros/local_position/pose", 
            PoseStamped, 
            self.local_pose_cb
        )

        self.global_pose_sub = rospy.Subscriber(
            self.topic_form + "/mavros/global_position/global",
            NavSatFix,
            self.global_pose_cb
        )

        self.local_vel_sub = rospy.Subscriber(
            self.topic_form + "/mavros/local_position/velocity_local",
            TwistStamped,
            self.local_vel_cb,
            queue_size=2,
        )

        self.global_vel_sub = rospy.Subscriber(
            self.topic_form + "/mavros/global_position/local",
            Odometry,
            self.global_vel_cb,
        )

        self.body_vel_sub = rospy.Subscriber(
            self.topic_form + "/mavros/local_position/velocity_body",
            TwistStamped,
            self.body_vel_cb,
        )
        
        self.acc_sub = rospy.Subscriber(
            self.topic_form + "/mavros/imu/data", 
            Imu, 
            self.acc_cb,
        )

        self.uav_state_sub = rospy.Subscriber(
            self.topic_form + "/mavros/state", 
            State, 
            self.state_cb
        )

        self.vel_control_sub = rospy.Subscriber(
            self.topic_form + "/control_signal/vector", 
            Vector3, 
            self.vel_control_cb
        )
        
        self.att_control_sub = rospy.Subscriber(
            self.topic_form + "/control_signal/att", 
            Float64MultiArray, 
            self.att_control_cb
        )

    def mavros_publisher(self):
        self.plane_att_pub = rospy.Publisher(
            self.topic_form + "/mavros/setpoint_raw/attitude",
            AttitudeTarget,
            queue_size=10,
        )
        self.plane_pos_pub = rospy.Publisher(
            self.topic_form + "/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=2,
        )
        self.plane_local_vel_pub = rospy.Publisher(
            self.topic_form + "/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=2,
        )

    def mavros_client(self):
        rospy.wait_for_service(self.topic_form + '/mavros/cmd/takeoff')
        self.takeoff_client = rospy.ServiceProxy(
            self.topic_form + '/mavros/cmd/takeoff', 
            CommandTOL
        )
        rospy.wait_for_service(self.topic_form + '/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy(
            self.topic_form + '/mavros/cmd/arming', 
            CommandBool
        )
        rospy.wait_for_service(self.topic_form + '/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy(
            self.topic_form + '/mavros/set_mode', 
            SetMode
        )



    def local_pose_cb(self, data):
        """Callback function to update position and orientation data."""
        self.local_pose = [
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
        ]
        self.orientation = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]

    def global_pose_cb(self, data):
        """Callback function to update latitude, longitude, altitude."""
        self.global_pose = [
            data.latitude,
            data.longitude,
            data.altitude,
        ]

    def local_vel_cb(self, data):
        """Callback function to update velocity in local EDU coordinates."""
        self.local_vel = [
            data.twist.linear.x,
            data.twist.linear.y,
            data.twist.linear.z,
        ]

    def global_vel_cb(self, data):
        """Callback function to update global velocity in END coordinates."""
        self.global_vel = [
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z,
        ]

    def body_vel_cb(self, data):
        """Callback function to update body velocity(front, left, up)."""
        self.body_vel = [
            data.twist.linear.x, 
            data.twist.linear.y, 
            data.twist.linear.z
        ]

    def acc_cb(self, data):
        """Callback function to update acceleration values."""
        """body_acc(back, right, down); global_acc(east, north, up);"""
        self.body_acc = [
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
        ]
        body_acc = np.copy(self.body_acc)
        q = self.orientation
        rot = quaternion_matrix(q)[:3,:3]
        self.global_acc = np.dot(rot,body_acc)
        self.global_acc[-1] = self.global_acc[-1] - 9.81
        

    def state_cb(self, data):
        self.state = data
        
    def vel_control_cb(self, msg):
        self.velocity_sp = [
            msg.x,
            msg.y,
            msg.z,
        ]
        self.control_mode = "vel"
    
    def att_control_cb(self, msg):
        self.attitude_sp = msg.data[0: 4]
        self.control_mode = "att"

    def takeoff(self):
        rospy.loginfo(self.global_pose)
        rospy.loginfo(self.state)
        target_distance = 100
        geod = Geodesic.WGS84
        g = geod.Direct(self.global_pose[0], 
                        self.global_pose[1],
                          90., target_distance)
        
        response = self.takeoff_client(
            min_pitch=0.0,
            yaw=90.,
            latitude=g['lat2'],
            longitude=g['lon2'],
            altitude=self.global_pose[2]+50-50
        )

        if response.success:
            rospy.loginfo("Takeoff initiated to target position: 30 meters forward.")
            # 调用解锁服务
            self.arming(True)
        else:
            rospy.logerr(f"Takeoff failed with result code: {response.result}")

    def arming(self, should_arm):
        """arming vehicle"""
        try:
            response = self.arming_client(value=should_arm)
            if response.success:
                rospy.loginfo("Arming successful" if should_arm else "Disarming successful")
            else:
                rospy.logerr("Arming failed")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_mode(self, mode):
        """Set the mode of the vehicle."""
        try:
            response = self.set_mode_client(0, mode)
            rospy.loginfo(self.state.mode)
            rospy.loginfo(response)
            if response.mode_sent:
                rospy.loginfo(f"Mode set to {mode} successfully.")
                return True
            else:
                rospy.logwarn(f"Failed to set mode to {mode}.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service 'set_mode' connection failed: {e}")
            return False

    def set_offboard(self):
        """Switch the vehicle to OFFBOARD mode."""
        if self.state.mode == "OFFBOARD":
            return "Already in OFFBOARD mode"
        
        if self.set_mode("OFFBOARD"):
            return "OFFBOARD mode set successfully"
        else:
            return "Attempting to set OFFBOARD mode"


    def MotionControl(self, control_type, control_vector, control_force=0.6):
        if control_type == "pos":
            control_pose = control_vector
            move_send = PoseStamped()
            move_send.pose.position.x = control_pose[0]
            move_send.pose.position.y = control_pose[1]
            move_send.pose.position.z = control_pose[2]
            self.plane_pos_pub.publish(move_send)

        elif control_type == "angular":
            control_attitude = control_vector
            move_send = AttitudeTarget()
            move_send.type_mask = move_send.IGNORE_ATTITUDE
            move_send.thrust = control_force
            move_send.body_rate.x = control_attitude[0]
            move_send.body_rate.y = control_attitude[1]
            move_send.body_rate.z = control_attitude[2]
            self.plane_att_pub.publish(move_send)
        elif control_type == "att":
            x, y, z, w = Eular2Quater(control_vector[0], control_vector[1], control_vector[2])
            move_send = AttitudeTarget()
            move_send.type_mask = move_send.IGNORE_ROLL_RATE \
                                + move_send.IGNORE_PITCH_RATE \
                                + move_send.IGNORE_YAW_RATE #+ move_send.IGNORE_THRUST
            move_send.orientation.x = x
            move_send.orientation.y = y
            move_send.orientation.z = z
            move_send.orientation.w = w
            move_send.thrust = control_force
            self.plane_att_pub.publish(move_send)

        elif control_type == "vel":
            control_velocity = control_vector[0:3]
            yaw = control_vector[-1]
            move_send = PositionTarget()
            move_send.type_mask = move_send.IGNORE_AFX \
                                + move_send.IGNORE_AFY \
                                + move_send.IGNORE_AFZ \
                                + move_send.IGNORE_PX  \
                                + move_send.IGNORE_PY  \
                                + move_send.IGNORE_PZ  \
                                + move_send.IGNORE_YAW_RATE  # + move_send.FORCE 
            move_send.coordinate_frame = move_send.FRAME_LOCAL_NED
            move_send.velocity.x = control_velocity[0]
            move_send.velocity.y = control_velocity[1]
            move_send.velocity.z = control_velocity[2]
            move_send.yaw = yaw
            self.plane_local_vel_pub.publish(move_send)

        else:
            rospy.logerr("No such control type in plane mode")

class VectorControlPlane(uav):
    def __init__(self, type = "plane", uav_index=0):
        super().__init__(type = "plane", uav_index=0)
        self.psi_pid = PID(
            0.05,
            np.pi/2.5,
            -np.pi/2.5,
            kp=1.,
            # ki=0.001,
            # kd=0.1,
            ki=0.1,
            kd=0.0,
            i_up_full=0.4,
            i_down_full=-0.4,
            d_up_full=0.5,
            d_down_full=-0.5,
            status_max=np.pi * 4.,
            status_min=-np.pi * 4.,
        )   # no limit
        self.pitch_pid = PID(
            0.05,
            np.pi/4.,
            -np.pi/4.,
            kp=1.,
            ki=0.01,
            kd=0.,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=5.,
            status_min=-5.,
        )   # no limit
        self.a_pid = PID(
            0.05,
            10.,
            -10.,
            kp=0.9,
            ki=0.01,
            kd=0.,
            i_up_full=0.5,
            i_down_full=-0.5,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=55.,
            status_min=10.,
        )   # no limit
        self.B_err_pid = PID(
            0.05,
            np.pi/6.,
            -np.pi/6.,
            kp=0.1,
            ki=0.002,
            kd=0.,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=1.,
            status_min=-1.,
        )

        self.E_err_pid = PID(
            0.05,
            1,
            -1,
            kp=0.1,
            ki=0.02,
            kd=0.,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=1.,
            status_min=-1,
        )

        self.pitch_h_pid = PID(
            0.05,
            np.pi/5.,
            -np.pi/5.,
            kp=0.2,
            ki=0.0,
            kd=0.0,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=1.,
            status_min=-1.,
        )

    def tecsControl(self, a_sp, desire_d_h, roll, dt):
        g = 9.81
        [u, v, w] = self.body_vel
        [ve, vn, vu] = self.local_vel
        [ae, an, au] = self.global_acc
        q = self.orientation
        v_horizontal = np.sqrt(ve**2+vn**2)
        a_horizontal = np.sqrt(ae**2+an**2)
        path_angel = np.arctan(1.0 * vu / v_horizontal)
        path_angel_sp = np.arctan(desire_d_h/v_horizontal)
        # energy calculate
        dot_B = path_angel - a_horizontal/g
        dot_E = a_horizontal/g + path_angel
        dot_B_sp = path_angel_sp - a_sp/g
        dot_E_sp = a_sp/g + 2.* path_angel_sp
        dot_B_err = dot_B_sp - dot_B
        dot_E_err = dot_E_sp - dot_E
        E_pi_out = self.E_err_pid.calculate(dot_E_sp, dot_E, dt)
        E_ff_out = 1. * dot_E_sp
        B_pi_out = self.B_err_pid.calculate(dot_B_sp, dot_B, dt)
        B_ff_out = 0.4 * dot_B_sp
        self.force = E_pi_out + E_ff_out + 0.5 + 0.0*np.abs(roll)
        desire_d_h = np.clip(desire_d_h, -5., 5.)
        pitch = np.arctan2(desire_d_h,v_horizontal) + self.pitch_h_pid.calculate(desire_d_h, vu, dt) 
        pitch = np.clip(pitch, -np.pi/6., np.pi/6.)

        if self.force < 0.15 or np.isnan(self.force):
            self.force = 0.15
        return self.force, pitch

    def Vec2Ori(self, desire_vec, dt=-1, method=1):
        if dt == -1:
            self.before_time = self.time_now
            self.time_now = time.time()
            dt = self.time_now - self.before_time
        vel_now = np.copy(self.local_vel)

        desire_h_d = desire_vec[2]  
        desire_psi = np.arctan2(desire_vec[1], desire_vec[0])  

        a = 0.9 * (np.linalg.norm(desire_vec) - np.linalg.norm(vel_now))
        v_now_xy = np.append(vel_now[[0, 1]], 0.)
        v_d_xy = np.append(desire_vec[[0, 1]], 0.)
        sig_vec = np.cross(v_now_xy, v_d_xy)

        if method == 0:
            k = 0.2
            a_t = desire_vec[[0, 1]] - desire_vec[[0, 1]] @ vel_now[[0, 1]] / np.linalg.norm(vel_now[[0, 1]])
            a_t *= k
            a_t = np.linalg.norm(a_t)
            desire_phi = np.arctan(a_t / 9.8) * np.sign(sig_vec[-1])
        elif method == 1:
            k = 0.5
            psi_diff = np.arccos(desire_vec[[0, 1]] @ vel_now[[0, 1]] / np.linalg.norm(vel_now[[0, 1]]) / np.linalg.norm(desire_vec[[0, 1]]))
            psi_diff = np.sign(sig_vec[-1]) * psi_diff
            psi_d = self.psi_pid.calculate(psi_diff - 0., 0., dt)
            desire_phi = np.arctan(np.linalg.norm(vel_now[[0, 1]]) * (psi_d) / 9.8)
            desire_phi = np.clip(desire_phi,-np.pi/4.,np.pi/4.)
        else: 
            pass

        return desire_h_d, desire_psi, -desire_phi, a
