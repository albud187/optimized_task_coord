#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import time
#message imports
from geometry_msgs.msg import Pose, Vector3Stamped, Twist, PoseStamped
from std_msgs.msg import String, Float64

r_id = rospy.get_namespace()
#subscribed topics
T_POSE = r_id+"pose_est"
#T_POSE = r_id+"pose_odom"
T_WP = r_id+"WP"
T_task = r_id + "task"
T_CMD_VEL = r_id+"cmd_vel"
#published topics
T_STATUS =r_id+"status"
T_USG = r_id+"usg"


#global variables
R_pose = Pose()

R_USG = Float64()

R_vel = Twist()
Last2Poses = []
R_USG = 0
#constants
r_id_raw = r_id[1:][:-1]

STATUSES = ["avb", "bus", "nsr" ]
#avb = available, bus = busy, nsr = not servicable

POS_TOL = 0.1

#functions

#functions
def handle_pose(msg):
    global R_pose 
    R_pose = msg
    


def handle_WP(msg):
    global R_WP
    R_WP = msg    

def handle_cmd_vel(msg):
    global R_vel
    R_vel = msg

def main_loop():
    #init
    
    global R_WP
    global R_USG
    global Last2Poses
    global R_pose
    rospy.init_node("robot_usage", anonymous = False)
    rate = rospy.Rate(10)
    
    #subscribers

    rospy.Subscriber(T_POSE, Pose, handle_pose)
    rospy.Subscriber(T_CMD_VEL, Twist, handle_cmd_vel)

    #publishers
    status_pub = rospy.Publisher(T_STATUS, String, queue_size = 10)
    usg_pub = rospy.Publisher(T_USG, Float64, queue_size = 10)

    #loop
    Last2Poses.append(R_pose)
    while not rospy.is_shutdown():
        
        Last2Poses.append(R_pose)
        if R_vel.linear.x !=0:
            ds = np.sqrt((Last2Poses[1].position.x - Last2Poses[0].position.x)**2+(Last2Poses[1].position.y -Last2Poses[0].position.y)**2)
            R_USG = R_USG + ds
        Last2Poses.pop(0)
        usg_pub.publish(R_USG)
        
        
        print(" ")
       
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass