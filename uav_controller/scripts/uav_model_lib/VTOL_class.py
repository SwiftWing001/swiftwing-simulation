#! /usr/bin/env python
import rospy
import numpy as np
import os
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandVtolTransition
from tf.transformations import euler_from_quaternion, quaternion_matrix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from myPID import PID
from useful_function import Orien2Eular, Velocity2Force, Eular2Quater

dir_mytest = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dir_mylib = dir_mytest + "/uav_swarm_lib"
sys.path.insert(0, dir_mylib)
print(dir_mylib)


class uav_vtol(object):
    def __init__(self, uav_index, uav_amount=2):
        self.type = "uav"  # this str will use in publisher/subscriber establish
        self.index = uav_index  # uav's index in group, every uav has a unique one
        if uav_amount == 1:
            self.topic_form = ""
        else: 
            self.topic_form = "/" + self.type + str(self.index)
        self.armed = False
        # self.mode = uav_state.land_origin
        self.mode = "Init"
        self.avoid = [0.0, 0.0, 0.0]
        self.pose = [0.0, 0.0, 0.0]           # [x,y,z]
        self.velocity = [0.0, 0.0, 0.0]       # [x,y,z]
        self.orientation = [0.0, 0.0, 0.0]    # [x,y,z,w]
        self.mavros_subscriber()
        self.mavros_client()
        self.mavros_publisher()
        self.force = 0.8
        self.before_time = time.time()
        self.time_now = time.time()
        self.fly_mode = "drone"
        rospy.loginfo("uav " + str(self.index) + " init success")

    def __del__(self):
        self.local_pose_sub.unregister()
        self.uav_state_sub.unregister()
        # self.motivation_pub.unregister()
        self.arm_client.close()
        self.set_mode_client.close()
        rospy.loginfo("uav " + str(self.index) + " delete success")

    def AvoidCB(self, msg):
        length = len(msg.layout.dim)
        for i in range(length):
            if str(self.index) == msg.layout.dim[i].label:
                index = msg.layout.dim[i].stride
                self.avoid[0] = msg.data[index]
                self.avoid[1] = msg.data[index + 1]
                self.avoid[2] = msg.data[index + 2]
                break

    def vel_CB(self, msg):
        self.velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def LocalPoseCB(self, data):
        self.pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.orientation = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]


    def mavros_subscriber(self):
        self.vel_sub = rospy.Subscriber(
            self.topic_form + "/mavros/local_position/velocity_local",
            TwistStamped,
            self.vel_CB,
            queue_size=2,
        )
        self.local_pose_sub = rospy.Subscriber(
            self.topic_form + "/vision_pose/pose", 
            PoseStamped, 
            self.LocalPoseCB
        )
        self.uav_state_sub = rospy.Subscriber(
            self.topic_form + "/mavros/state", 
            State, 
            self.StateCB
        )
        self.fly_state_sub = rospy.Subscriber(
            self.topic_form + "/mavros/extended_state",
            ExtendedState,
            self.FlyStateCB,
            queue_size=1,
        )
        self.global_vel_sub = rospy.Subscriber(
            self.topic_form + "/mavros/global_position/local",
            Odometry,
            self.globalVelCB,
        )
        self.acc_sub = rospy.Subscriber(
            self.topic_form + "/mavros/imu/data", Imu, self.accCB
        )
        self.velocity_sub = rospy.Subscriber(
            self.topic_form + "/mavros/local_position/velocity_body",
            TwistStamped,
            self.bodyVelCB,
        )

    def mavros_publisher(self):
        self.drone_pub = rospy.Publisher(
            self.topic_form + "/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=10,
        )
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
    def mavros_client(self):
        rospy.loginfo("waiting for service arming...")
        rospy.wait_for_service(self.topic_form + "/mavros/cmd/arming")
        self.arm_client = rospy.ServiceProxy(
            self.topic_form + "/mavros/cmd/arming", CommandBool
        )
        rospy.loginfo("waiting for service set_mode...")
        rospy.wait_for_service(self.topic_form + "/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(
            self.topic_form + "/mavros/set_mode", SetMode
        )
        rospy.loginfo("waiting for service vtol_transition...")
        rospy.wait_for_service(self.topic_form + "/mavros/cmd/vtol_transition")
        self.vtol_trans_client = rospy.ServiceProxy(
            self.topic_form + "/mavros/cmd/vtol_transition", CommandVtolTransition
        )

    def globalVelCB(self, data):
        """Callback function to update global velocity in END coordinates."""
        self.global_vel = [
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z,
        ]

    def accCB(self, data):
        """Callback function to update acceleration values."""
        """body_acc(back, right, down); global_acc(east, north, up);"""
        self.acc = [
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
        ]
        body_acc = np.copy(self.acc)
        q = self.orientation
        rot = quaternion_matrix(q)[:3,:3]
        self.global_acc = np.dot(rot,body_acc)
        self.global_acc[-1] = self.global_acc[-1] - 9.81

    def bodyVelCB(self, data):
        """Callback function to update body velocity(front, left, up)."""
        self.body_vel = [
            data.twist.linear.x, 
            data.twist.linear.y, 
            data.twist.linear.z
        ]

    def StateCB(self, data):
        self.armed = data.armed
        self.mode = data.mode

    def FlyStateCB(self, data):
        """get the vtol's fly mode"""
        if data.vtol_state == 4:
            self.fly_mode = "plane"
        elif data.vtol_state == 3:
            self.fly_mode = "drone"
        elif data.vtol_state == 2:
            self.fly_mode = "plane2drone"
        elif data.vtol_state == 1:
            self.fly_mode = "drone2plane"
        else:
            rospy.logerr("uav " + str(self.index) + " fly mode wrong!")

    def ModeSet(self, mode):
        try:
            answer = self.set_mode_client(0, mode)
            return answer.mode_sent
        except:
            rospy.logerr("service: 'set_mode' connect fail!")
            return -1

    def Arming(self, arm_command):
        try:
            answer = self.arm_client(arm_command)
            return answer.success
        except:
            rospy.logerr("service: 'arming' connect fail! ")
            return -1

    def FlyModeTrans(self, mode):
        if not (self.fly_mode == "drone" and mode == "plane") or \
            (self.fly_mode == "plane" and mode == "drone"):
            rospy.logwarn("Can't trans mode")
            return -1
            
        header = Header()
        try:
            if self.fly_mode == "drone" or self.fly_mode == "plane2drone":
                answer = self.vtol_trans_client(header, 4)
            else:
                answer = self.vtol_trans_client(header, 3)
            return answer.success
        except:
            rospy.logerr("service: 'vtol_transition' connect fail!")

    def LandOrigin(self):
        if self.mode == "AUTO.RTL":
            return "success"
        set_answer = self.ModeSet("AUTO.RTL")
        return "trying"

    def Manual(self):
        if self.mode == "OFFBOARD":
            return "success"
        set_answer = self.ModeSet("OFFBOARD")
        # print(set_answer)
        return "trying"

    def MotionControl(self, control_type, control_vector, control_force=0.6):
        # if your px4 version is 1.13 or earlier version,please use the following sentence:
        # if self.fly_mode == "drone" or self.fly_mode == "drone2plane":
        if self.fly_mode == "drone": 
            move_send = PositionTarget()
            if control_type == "pos":
                move_send.type_mask = (
                    PositionTarget.IGNORE_VX
                    + PositionTarget.IGNORE_VY
                    + PositionTarget.IGNORE_VZ
                    + PositionTarget.IGNORE_AFX
                    + PositionTarget.IGNORE_AFY
                    + PositionTarget.IGNORE_AFZ
                    + PositionTarget.IGNORE_YAW_RATE
                )
                move_send.coordinate_frame = 1
                move_send.position.x = control_vector[0]
                move_send.position.y = control_vector[1]
                move_send.position.z = control_vector[2]
                # move_send.yaw = control_vector[3]
            elif control_type == "vel":
                move_send.type_mask = (
                    PositionTarget.IGNORE_PX
                    + PositionTarget.IGNORE_PY
                    + PositionTarget.IGNORE_PZ
                    + PositionTarget.IGNORE_AFX
                    + PositionTarget.IGNORE_AFY
                    + PositionTarget.IGNORE_AFZ
                    + PositionTarget.IGNORE_YAW
                )
                move_send.coordinate_frame = 8
                move_send.velocity.x = control_vector[0]
                move_send.velocity.y = control_vector[1]
                move_send.velocity.z = control_vector[2]
                # move_send.yaw = -3.14159/2
                move_send.yaw = 0
            elif control_type == "acc":
                move_send.type_mask = (
                    PositionTarget.IGNORE_PX
                    + PositionTarget.IGNORE_PY
                    + PositionTarget.IGNORE_PZ
                    + PositionTarget.IGNORE_VX
                    + PositionTarget.IGNORE_VY
                    + PositionTarget.IGNORE_VZ
                    + PositionTarget.IGNORE_YAW
                )
                move_send.acceleration_or_force.x = control_vector[0]
                move_send.acceleration_or_force.y = control_vector[1]
                move_send.acceleration_or_force.z = control_vector[2]
            else:
                rospy.logwarn("No such control type!(rotor)")
            self.drone_pub.publish(move_send)
        # elif self.fly_mode == "plane":
        elif self.fly_mode == "plane":
            if control_type == "pos":
                control_pose = control_vector
                move_send = PoseStamped()
                move_send.pose.position.x = control_pose[0]
                move_send.pose.position.y = control_pose[1]
                move_send.pose.position.z = control_pose[2]
                self.plane_pos_pub.publish(move_send)

            elif control_type == "vel":
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
                move_send.type_mask = move_send.IGNORE_ROLL_RATE + move_send.IGNORE_PITCH_RATE + move_send.IGNORE_YAW_RATE #+ move_send.IGNORE_THRUST
                move_send.orientation.x = x
                move_send.orientation.y = y
                move_send.orientation.z = z
                move_send.orientation.w = w
                move_send.thrust = control_force
                self.plane_att_pub.publish(move_send)
            else:
                rospy.logerr("No such control type in plane mode")


class VectorControlVtol(uav_vtol):
    # input: desire velocity vector
    # output: desire orientation
    def __init__(self, uav_index, uav_amount):
        super().__init__(uav_index, uav_amount)
        self.psi_pid = PID(
            0.05,
            np.pi/2.,
            -np.pi/2.,
            kp=0.5,
            ki=0.05,
            kd=0.0,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=np.pi * 4.,
            status_min=-np.pi * 4.,
        )   # no limit
        self.pitch_pid = PID(
            0.05,
            np.pi/4.,
            -np.pi/4.,
            kp=0.1,
            ki=0.03,
            kd=0.,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=5.,
            status_min=-5.,
        )
        self.a_pid = PID(
            0.05,
            10.,
            -10.,
            kp=2.,
            ki=0.1,
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
            1.,
            -1.,
            kp=0.1,
            ki=0.1,
            kd=0.,
            i_up_full=1.,
            i_down_full=-1.,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=1.,
            status_min=-1.,
        )

        self.E_err_pid = PID(
            0.05,
            1,
            -1,
            kp=0.05,
            ki=0.02,
            kd=0.,
            i_up_full=0.2,
            i_down_full=-0.2,
            d_up_full=0.1,
            d_down_full=-0.1,
            status_max=1.,
            status_min=-1,
        )
    
    def tecsControl(self, a, desire_v, roll, dt):
        g = 9.81
        [u, v, w] = self.velocity
        [ve, vn, vd] = self.global_vel
        [ax, ay, az] = self.acc
        v_horizontal = np.sqrt(ve**2+vn**2)
        desire_d_h = desire_v[2]

        v_real = np.linalg.norm(self.global_vel)
        v_desire = np.linalg.norm(desire_v)
        a_real = np.linalg.norm(self.acc)
        dot_E = a_real / g + self.velocity[2] / v_real
        dot_E_sp = a / g + desire_d_h / v_desire 
        dot_E_err = dot_E_sp - dot_E

        E_pi_out = self.E_err_pid.calculate(dot_E_sp, dot_E, dt)
        E_ff_out = 1. * dot_E_sp
        self.force = E_pi_out + E_ff_out + 0.5
        dot_B = np.arcsin(self.global_vel[2] / v_real) - a_real / g
        dot_B_sp = np.arcsin(desire_v[2] / v_desire) - a / g
        dot_B_err = dot_B_sp - dot_B
        B_pi_out = self.B_err_pid.calculate(dot_B_sp, dot_B, dt)
        B_ff_out = 0.1 * dot_B
        pitch = B_pi_out + B_ff_out - 0.2 
        pitch = self.pitch_pid.calculate(desire_d_h, 0., dt) + 0.0*np.abs(roll)
        pitch = np.clip(pitch, -np.pi/6, np.pi/5)
        if self.force < 0.15 or np.isnan(self.force):
            self.force = 0.15
        return self.force, pitch

    def Vec2Ori(self, desire_vec, dt=-1, method=0):
        if dt == -1:
            self.before_time = self.time_now
            self.time_now = time.time()
            dt = self.time_now - self.before_time
        vel_now = np.copy(self.velocity)
        desire_h_d = desire_vec[2] 
        desire_psi = np.arctan2(desire_vec[1], desire_vec[0])
        a = self.a_pid.calculate(np.linalg.norm(desire_vec), np.linalg.norm(vel_now), dt)
        v_now_xy = np.append(vel_now[[0, 1]], 0.)
        v_d_xy = np.append(desire_vec[[0, 1]], 0.)
        sig_vec = np.cross(v_now_xy, v_d_xy)

        if method == 0:
            k = 0.2
            a_t = desire_vec[[0, 1]] - desire_vec[[0, 1]] @ vel_now[[0, 1]] / np.linalg.norm(vel_now[[0, 1]])
            a_t *= k
            a_t = np.linalg.norm(a_t)
            desire_phi = np.arctan(a_t / 9.8) * np.sign(sig_vec[-1])
        else:
            k = 0.5
            if np.linalg.norm(vel_now[[0, 1]]) <=0.1**8 or np.linalg.norm(desire_vec[[0, 1]]) <= 0.1**8:
                psi_diff = 0.
            else:
                psi_diff = np.arccos(desire_vec[[0, 1]] @ vel_now[[0, 1]] / np.linalg.norm(vel_now[[0, 1]]) / np.linalg.norm(desire_vec[[0, 1]]))
                psi_diff = np.sign(sig_vec[-1]) * psi_diff
            psi_d = self.psi_pid.calculate(psi_diff - 0., 0., dt)
            desire_phi = np.arctan(np.linalg.norm(vel_now[[0, 1]]) * psi_d / 9.8)
            desire_phi = np.clip(desire_phi, -np.pi/4, np.pi/4)

        return desire_h_d, desire_psi, -desire_phi, a