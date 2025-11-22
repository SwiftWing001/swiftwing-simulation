#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import sys
from gazebo_msgs.msg import ModelStates

try:
    vehicle_type = rospy.get_param("uav_type")
except KeyError:
    if len(sys.argv) >= 3:
        vehicle_type = sys.argv[1]
    else:
        vehicle_type = "standard_vtol"
    rospy.logwarn("Start without'uav_type', set uav_type=" + vehicle_type)
try:
    vehicle_num = int(rospy.get_param("uav_amount"))
except KeyError:
    if len(sys.argv) >= 3:
        vehicle_num = int(sys.argv[2])
    else:
        vehicle_num = 1
    rospy.logwarn("Start without 'uav_amount', set uav_amount = " + str(vehicle_num))
multi_pose_pub = [None]*(vehicle_num)
multi_speed_pub = [None]*vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_speed = [TwistStamped() for i in range(vehicle_num)]

def gazebo_model_state_callback(msg):
    for vehicle_id in range(vehicle_num):
        if vehicle_num == 1:
            id = msg.name.index(vehicle_type)
        else: 
            id = msg.name.index(vehicle_type+str(vehicle_id))
        multi_local_pose[vehicle_id].header.stamp = rospy.Time().now()
        multi_local_pose[vehicle_id].header.frame_id = 'map'
        multi_local_pose[vehicle_id].pose = msg.pose[id]
        multi_speed[vehicle_id].header.stamp = rospy.Time().now()
        multi_speed[vehicle_id].header.frame_id = 'map'
        multi_speed[vehicle_id].twist = msg.twist[id]


if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_get_pose_groundtruth')
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)

    if vehicle_num == 1:
        multi_pose_pub[0] = rospy.Publisher('/vision_pose/pose', PoseStamped, queue_size=1)
        multi_speed_pub[0] = rospy.Publisher('/vision_speed/speed', TwistStamped, queue_size=1)
    else: 
        for i in range(vehicle_num):
            multi_pose_pub[i] = rospy.Publisher("/uav"+str(i)+'/vision_pose/pose', PoseStamped, queue_size=1)
            multi_speed_pub[i] = rospy.Publisher("/uav"+str(i)+'/vision_speed/speed', TwistStamped, queue_size=1)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            multi_pose_pub[i].publish(multi_local_pose[i])
            multi_speed_pub[i].publish(multi_speed[i])
        try:
            rate.sleep()
        except:
            continue

