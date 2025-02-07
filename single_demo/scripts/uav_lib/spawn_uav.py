#!/usr/bin/env python3
import rospy
import numpy as np
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


class uav():
    def __init__(self, type = "uav", uav_amount=1, uav_index=0):
        self.type = type        # this str will use in publisher/subscriber establish
        self.index = uav_index  # uav's index in group, every uav has a unique one
        if uav_amount > 1:
            self.topic_form = "/" + self.type + str(self.index)
        else: 
            self.topic_form = ""
        self.armed = False
        self.state = State()
        self.avoid = [0.0, 0.0, 0.0]
        self.body_acc = [0.1, 0.1, 0.1]
        self.global_acc = [0.1, 0.1, 0.1]
        self.local_pose = [0.1, 0.1, 0.1]
        self.global_pose = [0.1, 0.1, 0.1]
        self.gazebo_pose = [0.1, 0.1, 0.1]
        self.local_vel = [0.1, 0.1, 0.1]
        self.global_vel = [0.1, 0.1, 0.1] 
        self.body_vel = [0.1, 0.1, 0.1] 
        self.orientation = [0.1, 0.1, 0.1, 0.1]  
        self.gazebo_orientation = [0.1, 0.1, 0.1, 0.1]
        self.mavros_subscriber()
        self.mavros_publisher()

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
        
        self.gazebo_pose_sub = rospy.Subscriber(
            self.topic_form + "/vision_pose/pose", 
            PoseStamped, 
            self.gazebo_pose_cb
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

    def mavros_publisher(self):
        self.vel_control_pub = rospy.Publisher(
            self.topic_form + '/control_signal/vector',
            Vector3,
            queue_size=2
        )
        self.att_control_pub = rospy.Publisher(
            self.topic_form + '/control_signal/att',
            Float64MultiArray,
            queue_size=2
        )
    
    def control_send(self, cmd, cmd_type):
        if cmd_type == "vel":
            cmd_send = Vector3()
            cmd_send.x = cmd[0]
            cmd_send.y = cmd[1]
            cmd_send.z = cmd[2]
            self.vel_control_pub.publish(cmd_send)
        elif cmd_type == "att":
            cmd_send = Float64MultiArray()
            cmd_send.data = [cmd[0], cmd[1], cmd[2], cmd[3]]    # [roll, pitch, yaw, force]
            self.att_control_pub.publish(cmd_send)
        else:
            raise ValueError("No such cmd type!")

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

    def gazebo_pose_cb(self, data):
        self.gazebo_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.gazebo_orientation = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
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
        body_acc = self.body_acc
        q = self.orientation
        rot = quaternion_matrix(q)[:3,:3]
        self.global_acc = np.dot(rot,body_acc)
        self.global_acc[-1] = self.global_acc[-1] - 9.81

    def state_cb(self, data):
        self.state = data
