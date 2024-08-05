#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import time
#message imports
from geometry_msgs.msg import Twist,  Vector3Stamped, PoseStamped, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

R_ID = rospy.get_namespace()

#subscribed topics
T_POSE = R_ID+"pose_est"
#T_POSE = R_ID+"pose_odom"
T_VEL_D = R_ID+"vel_d"
T_WP = R_ID+"WP"
T_TASK = R_ID +"task"
#published topics
T_CMD_VEL = R_ID+"cmd_vel"
T_VEL_A = R_ID+"vel_a"

#global variables
tgt_vel = Vector3Stamped()
zero_vel = Twist()
R_pose = Pose()
R_WP = Vector3Stamped()
R_task = PoseStamped()
POS_TOL = 0.05
VEL_TOL = 0.03
#functions
def handle_pose(msg):
    global R_pose
    R_pose = msg

def handle_vel(msg):
    global tgt_vel
    tgt_vel = msg

def handle_wp(msg):
    global R_WP
    R_WP = msg

def handle_task(msg):
    global R_task
    R_task = msg

def yaw_rate(error_angle):
    Kw = 1.82
    kt = 1
    result = 0
    if abs(error_angle) >= np.pi:
        if error_angle > 0:
            error_angle -= (2 * np.pi)
        else:
            error_angle += (2 * np.pi)

    if abs(error_angle) >= np.pi/3:
        result= Kw*np.sin(kt*error_angle+0.0001)/abs(np.sin(kt*error_angle+0.0001))
    else:
        result = Kw*np.sin(kt*error_angle)
    
    return result

    

def CMD_VEL(tgt_vel, r_pose):
    """
    inputs: tgt_vel Vector3Stamped - desired velocity in global frame in x and y components. r_pose Pose - robot's current pose
    outputs: result_vel Twist - command velocity in terms of control inputs
    Description: converts a velocity in global frame into command velocity in terms of robot's control inputs
    """
    Kw = 1.82
    
    result_vel = Twist()
    vx = tgt_vel.vector.x
    vy = tgt_vel.vector.y

    v_mag = np.sqrt(vx**2+vy**2)

    v_theta = np.arctan2(vy,vx)
    error_angle = v_theta  - r_pose.orientation.z

    
    result_vel.angular.z = yaw_rate(error_angle)
    if abs(error_angle) <= np.pi/3:
        result_vel.linear.x = v_mag*np.cos(error_angle)
    else:
        result_vel.linear.x = 0
    return result_vel

def actual_vel(R_pose, cmd_vel):
    result_vel = Vector3Stamped()
    result_vel.vector.x = cmd_vel.linear.x * np.cos(R_pose.orientation.z)
    result_vel.vector.y = cmd_vel.linear.x * np.sin(R_pose.orientation.z)
    return result_vel

def distance_wp(R_pose, R_WP):
    distance = np.sqrt((R_pose.position.x -R_WP.vector.x)**2+(R_pose.position.y -R_WP.vector.y)**2)
    return distance

#main loop
def main_loop():
    
    global R_pose
    global R_WP
    global tgt_vel
    global R_task
    rospy.init_node("velocity_publisher", anonymous = False)
    rate = rospy.Rate(30)
    
    
    #subscribers
    rospy.Subscriber(T_POSE, Pose, handle_pose)
    rospy.Subscriber(T_VEL_D, Vector3Stamped, handle_vel)
    rospy.Subscriber(T_WP, Vector3Stamped, handle_wp)
    rospy.Subscriber(T_TASK, PoseStamped, handle_task)
    #publishers
    R_vel_pub = rospy.Publisher(T_CMD_VEL, Twist, queue_size = 10)
    A_vel_pub = rospy.Publisher(T_VEL_A, Vector3Stamped, queue_size=10)

    while not rospy.is_shutdown():
        
        #if distance_wp(R_pose, R_WP)>POS_TOL or np.sqrt(tgt_vel.vector.x**2+tgt_vel.vector.y**2)>VEL_TOL:
        if distance_wp(R_pose, R_WP)>POS_TOL and R_task.pose.orientation.w ==1:
            print(R_vel)
            R_vel = CMD_VEL(tgt_vel, R_pose)
        else:
            R_vel = zero_vel

        R_vel_pub.publish(R_vel)

        R_vel_a = actual_vel(R_pose, R_vel)
        A_vel_pub.publish(R_vel_a)
 
        rate.sleep()
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass