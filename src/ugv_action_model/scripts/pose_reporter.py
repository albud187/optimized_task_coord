#!/usr/bin/env python3

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os

#message imports
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

#subscribed topics
TB3_ID = rospy.get_namespace()

T_ODO = TB3_ID+"/odom"

#published topics
T_POSE = TB3_ID+"/pose_est"

#global variables
WB_pose = Pose()

#functions
def handle_pose(odo_msg):
    """
    input: odo_msg Odometry - robot's pose estiamte from odomoetry
    output: result Pose - robot's pose estimate in terms of pose data type
    Description: converts datatype from odometry to Pose, and experesses orientation in roll, pitch, yaw instead of quaternions
    """
    result = Pose()
    result.position.x = odo_msg.pose.pose.position.x
    result.position.y = odo_msg.pose.pose.position.y
    result.position.z = odo_msg.pose.pose.position.z

    pose_quat = [odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w]

    (result.orientation.x, result.orientation.y, result.orientation.z) = euler_from_quaternion(pose_quat)
    return result
    
def handle_WB_pose(msg):
    global WB_pose
    WB_pose = handle_pose(msg)
  
#main loop
def main_loop():
    global WB_pose

    
    rospy.init_node("pose_reporter_sim", anonymous = False)
    rate = rospy.Rate(500)
    #subscribers
    rospy.Subscriber(T_ODO, Odometry, handle_WB_pose)

    print("test")
    #publishers
    Pose_pub = rospy.Publisher(T_POSE, Pose, queue_size = 10)
 
    while not rospy.is_shutdown():
        print(WB_pose)
        Pose_pub.publish(WB_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass