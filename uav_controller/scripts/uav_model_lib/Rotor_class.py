#!/usr/bin/env python3
import rospy
import numpy as np
import time
import os
import sys
from tf.transformations import euler_from_quaternion, quaternion_matrix
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from pygeodesy.geoids import GeoidPGM
dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/script/uav_model_lib"
sys.path.insert(0, dir_mylib)
print(dir_mylib)

from myPID import PID
from useful_function import Eular2Quater

class uav():
    def __init__(self, type = "rotor", uav_index=None):
        self.type = type        # this str will use in publisher/subscriber establish
        self.index = uav_index  # uav's index in group, every uav has a unique one
        if self.index == None:
            self.topic_form = ""
        else:
            self.topic_form = "/" + self.type + str(self.index)
        self.armed = False
        self.state = State()

        self.mode = "Init"
        self.avoid = [0.0, 0.0, 0.0]

        self.body_acc = [0.1, 0.1, 0.1]
        self.global_acc = [0.1, 0.1, 0.1]
        self.local_pose = [0.1, 0.1, 0.1]
        self.gps_pose = [0.1, 0.1, 0.1] 
        if self.index == None:
            self.global_pose = [0.1, 0.1, 0.1]
        else:
            self.global_pose = self.local_pose

        self.local_vel = [0.1, 0.1, 0.1]
        if self.index == None:
            self.global_vel = [0.1, 0.1, 0.1] 
        else:
            self.global_vel = self.local_vel

        self.body_vel = [0.1, 0.1, 0.1] 
        self.orientation = [0.1, 0.1, 0.1, 0.1]
        self.velocity_sp = [0.1, 0.1, 0.1, 0.1]
        self.R_enu2ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.R_local2global = np.array([[1., 0, 0], [0, 1., 0], [0, 0, 1.]])
        self.time_now = time.time()  
        self.mavros_subscriber()
        self.mavros_client()
        self.mavros_publisher()
        self.control_mode = "none"

    def mavros_subscriber(self):
        self.gps_pose_sub = rospy.Subscriber(
            self.topic_form + "/mavros/global_position/global",
            NavSatFix,
            self.global_pose_cb
        )
        if self.index == None:
            self.local_pose_sub = rospy.Subscriber(
                self.topic_form + "/mavros/local_position/pose", 
                PoseStamped, 
                self.local_pose_cb
            )
            # self.global_pose_sub = rospy.Subscriber(
            #     self.topic_form + "/mavros/global_position/global",
            #     NavSatFix,
            #     self.global_pose_cb
            # )

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
        else:
            self.local_pose_sub = rospy.Subscriber(
                self.topic_form + "/mavros/vision_pose/pose", 
                PoseStamped, 
                self.local_pose_cb
            )
            self.local_vel_sub = rospy.Subscriber(
                self.topic_form + "/mavros/vision_speed/speed",
                TwistStamped,
                self.local_vel_cb,
                queue_size=2,
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
        
        # self.att_control_sub = rospy.Subscriber(
        #     self.topic_form + "/control_signal/att", 
        #     Float64MultiArray, 
        #     self.att_control_cb
        # )

    def mavros_publisher(self):
        self.rotor_cmd_pub = rospy.Publisher(
            self.topic_form + "mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=2,
        )
        """
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
        """

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

        x = self.orientation[0]
        y = self.orientation[1]
        z = self.orientation[2]
        w = self.orientation[3]
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ])
        self.R_local2global = R.T

    def global_pose_cb(self, data):
        """Callback function to update latitude, longitude, altitude."""
        self.gps_pose = [
            data.latitude,
            data.longitude,
            data.altitude,
        ]
        if self.index == None:
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
    
    # def att_control_cb(self, msg):
    #     self.attitude_sp = msg.data[0: 4]
    #     self.control_mode = "att"

    def takeoff(self, h=10.):
        rospy.loginfo(self.gps_pose)
        rospy.loginfo(self.local_pose)
        rospy.loginfo(self.state)
        
        rospy.loginfo("Try to takeoff...")
        if self.state.armed == False:
            rospy.loginfo("Arming...")
            # 调用解锁服务
            arm_answer = self.arming(True)
            if arm_answer == False:
                rospy.logerr("Fail to takeoff, because arming failure!")
                return False
        time.sleep(0.1)
        
        # 计算高度差
        _egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        offset_height =  _egm96.height(self.gps_pose[0], self.gps_pose[1])
        
        response = self.takeoff_client(
            min_pitch=0.0,
            yaw=90.,
            latitude=self.gps_pose[0],
            longitude=self.gps_pose[1],
            altitude=self.gps_pose[2] - offset_height + h,
        )
        if response.success:
            return True
        else:
            rospy.logerr("Fail to takeoff!")
            return False


    def arming(self, should_arm):
        """arming vehicle"""
        try:
            response = self.arming_client(value=should_arm)
            if response.success:
                rospy.loginfo("Arming successful" if should_arm else "Disarming successful")
                return True
            else:
                rospy.logerr("Arming failed")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

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
        move_send = PositionTarget()
        vec = np.array(control_vector[0:3])
        if control_type == "vel" or control_type == "acc":
            vec = self.R_local2global @ vec

        vec_temp = self.R_enu2ned @ np.array(self.global_vel)
        v_x = self.global_vel[0]
        v_y = self.global_vel[1]
        v_z = self.global_vel[2]
        # v_x = vec_temp[0]
        # v_y = vec_temp[1]
        # v_z = vec_temp[2]
        if np.linalg.norm([v_x, v_y, v_z]) > 0.05:
            # yaw_cmd = -np.arccos(v_x / np.sqrt(v_x**2 + v_y**2)) - np.pi / 2.
            # yaw_cmd = np.arccos(v_x / np.sqrt(v_x**2 + v_y**2))
            yaw_cmd = np.arctan2(v_y, v_x) - np.pi / 2.
            # if yaw_cmd < 0:
            #     yaw_cmd += 2 * np.pi
            print("v_x is: ", v_x, "yaw is: ", yaw_cmd)
        else:
            yaw_cmd = 0.
        # yaw_cmd = np.pi / 4.
        if control_type == "pos":
            move_send.type_mask = (
                PositionTarget.IGNORE_VX
                + PositionTarget.IGNORE_VY
                + PositionTarget.IGNORE_VZ
                + PositionTarget.IGNORE_AFX
                + PositionTarget.IGNORE_AFY
                + PositionTarget.IGNORE_AFZ
                # + PositionTarget.IGNORE_YAW
                + PositionTarget.IGNORE_YAW_RATE
             )
            move_send.coordinate_frame = 1
            move_send.position.x = vec[0]
            move_send.position.y = vec[1]
            move_send.position.z = vec[2]
            move_send.yaw = yaw_cmd
            self.rotor_cmd_pub.publish(move_send)
        elif control_type == "vel":
            move_send.type_mask = (
            PositionTarget.IGNORE_PX
                + PositionTarget.IGNORE_PY
                + PositionTarget.IGNORE_PZ
                + PositionTarget.IGNORE_AFX
                + PositionTarget.IGNORE_AFY
                + PositionTarget.IGNORE_AFZ
                # + PositionTarget.IGNORE_YAW
                + PositionTarget.IGNORE_YAW_RATE
            )
            move_send.coordinate_frame = 8
            move_send.velocity.x = vec[0]
            move_send.velocity.y = vec[1]
            move_send.velocity.z = vec[2]
            move_send.yaw = yaw_cmd
            self.rotor_cmd_pub.publish(move_send)
        elif control_type == "acc":
            move_send.type_mask = (
                PositionTarget.IGNORE_PX
                + PositionTarget.IGNORE_PY
                + PositionTarget.IGNORE_PZ
                + PositionTarget.IGNORE_VX
                + PositionTarget.IGNORE_VY
                + PositionTarget.IGNORE_VZ
                # + PositionTarget.IGNORE_YAW
                + PositionTarget.IGNORE_YAW_RATE
            )
            move_send.acceleration_or_force.x = vec[0]
            move_send.acceleration_or_force.y = vec[1]
            move_send.acceleration_or_force.z = vec[2]
            move_send.yaw = yaw_cmd
            self.rotor_cmd_pub.publish(move_send)
        else:
            rospy.logerr("No such control type in plane mode")

