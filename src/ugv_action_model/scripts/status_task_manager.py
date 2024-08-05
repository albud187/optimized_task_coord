#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import time
#message imports
from geometry_msgs.msg import Pose, Vector3Stamped, Twist, PoseStamped
from std_msgs.msg import String, Float64, Int16

r_id = rospy.get_namespace()
#subscribed topics
T_POSE = r_id+"pose_est"
#T_POSE = r_id+"pose_odom"

T_WP = r_id+"WP"
T_task = r_id + "task"
T_alloc_task = r_id+"/alloc_task"

#published topics
T_STATUS =r_id+"robot_status"
T_USG = r_id+"usg"
T_CMD_VEL = r_id+"cmd_vel"
T_comp_task = "/comp_task"

#global variables
R_pose = Pose()
start_pose = Pose()
R_task = PoseStamped()
R_WP = Vector3Stamped()
R_USG = Float64()
R_status = String()
R_vel = Twist()
Last2Poses = []
R_USG = 0
alloc_task_idx = -1
#constants
r_id_raw = r_id[1:][:-1]
prev_param_seq = 0

STATUSES = ["avb", "bus", "nsr" ]
#avb = available, bus = busy, nsr = not servicable

POS_TOL = 0.1

#callbacks
def handle_task(msg):
    global R_task
    global start_pose
    global R_pose
    R_task = msg
    start_pose = R_pose

def handle_pose(msg):
    global R_pose
    R_pose = msg

#saves the idx of allocated task
def handle_alloc(msg):
    global alloc_task_idx
    alloc_task_idx = msg
#functions

def check_update_task(task_param):
    global prev_param_seq
    if task_param.header.seq != prev_param_seq:
        result = True
    else:
        result = False
    prev_param_seq = task_param.header.seq
    return result
TASK_COMP = 0

def patrol_task(WP_pub, status_pub, comp_pub):
    global R_WP
    global TASK_COMP
    global R_status
    global R_task
    global R_pose
    global alloc_task_idx
    r_status = String()
    r = R_task.pose.orientation.x/8
    pcx = R_task.pose.position.x
    pcy = R_task.pose.position.y
    wp1, wp2, wp3, wp4 = Vector3Stamped(), Vector3Stamped(), Vector3Stamped(), Vector3Stamped()
    
    wp1.vector.x,wp1.vector.y  = pcx-r, pcy-r
    wp2.vector.x,wp2.vector.y  = pcx-r, pcy+r
    wp3.vector.x,wp3.vector.y  = pcx+r, pcy+r
    wp4.vector.x,wp4.vector.y  = pcx+r, pcy-r
    waypoints =[wp1, wp2, wp3, wp4, wp1]
   

    wp_idx = 0
    
    while wp_idx < len(waypoints) and TASK_COMP!=1:
       
        r_status.data = "bus"
        status_pub.publish(r_status)
        wp = waypoints[wp_idx]
        
        WP_pub.publish(wp)
        distance = np.sqrt((wp.vector.x-R_pose.position.x)**2+(wp.vector.y-R_pose.position.y)**2)
        if distance < 0.1:
            wp_idx = wp_idx + 1
        TASK_COMP = 0

        if wp_idx == len(waypoints):
            TASK_COMP =1
    else:
        TASK_COMP = 1
        
        r_status.data = "avb"
        status_pub.publish(r_status)
        comp_task_idx = Int16()
        comp_task_idx.data = alloc_task_idx
        comp_pub.publish(comp_task_idx)
        return None

def go2goal(start_pose, R_pose, WP_pub, status_pub, comp_pub):
    """
    inputs: Pose R_pose, Publisher WP_pub
    outputs: none
    description: publishes waypoints to the robot. The robot moves to the waypoint(s).
    Task complete when robot is within POS_TOL distance of last waypoint
    returns false if task is incomplete
    Uses R_task as a global variable
    """

    global R_WP
    global TASK_COMP
    global alloc_task_idx
    r_status = String()
    print(start_pose)
    #do task here
    
    R_WP.vector.x  = R_task.pose.position.x
    R_WP.vector.y = R_task.pose.position.y

    WP_pub.publish(R_WP)
    distance_to_WP = np.sqrt((R_pose.position.x - R_WP.vector.x)**2+(R_pose.position.y - R_WP.vector.y)**2)
    print(distance_to_WP)
    print(" ")
    ctr = 0
    #check task completion criteria
    if distance_to_WP <= POS_TOL:
        if TASK_COMP!=1:
            while ctr <3:
                time.sleep(0.2)
                ctr+=1
                print("count: " +str(ctr))

            print("task complete")
            TASK_COMP = 1
            r_status.data = "avb"
            status_pub.publish(r_status)
            comp_task_idx = Int16()
            comp_task_idx.data = alloc_task_idx
            comp_pub.publish(comp_task_idx)
    if distance_to_WP > POS_TOL:
        TASK_COMP = 0
        r_status.data = "bus"
        status_pub.publish(r_status)


def main_loop():
    #init
    
    global R_task
    global TASK_COMP
    global R_pose
    global counter
    global start_pose
    rospy.init_node("status_task_manager", anonymous = False)
    rate = rospy.Rate(10)
    
    #subscribers
    rospy.Subscriber(T_task, PoseStamped, handle_task)
    rospy.Subscriber(T_POSE, Pose, handle_pose)
    rospy.Subscriber(T_alloc_task, Int16, handle_alloc)
    #publishers
    status_pub = rospy.Publisher(T_STATUS, String, queue_size = 10)
    WP_pub = rospy.Publisher(T_WP, Vector3Stamped, queue_size = 10)
    comp_task_pub = rospy.Publisher(T_comp_task, Int16, queue_size=10)
    #loop
    R_status.data = STATUSES[0]
    while not rospy.is_shutdown():
        status_pub.publish(R_status)

        if check_update_task(R_task) == True: 
            TASK_COMP = 0
            print("recieved new task")
            print(R_task)
            while R_task.pose.orientation.w != 0 and TASK_COMP == 0:
                
                rate.sleep()
                #ground patrol task:
                if R_task.header.frame_id == "D":
                    print(r_id+" - patrol")
                    time.sleep(0.15)
                    patrol_task(WP_pub, status_pub, comp_task_pub)
                #go to goal
                if R_task.header.frame_id == "G" or R_task.header.frame_id=="E" or R_task.header.frame_id=="F" or R_task.header.frame_id=="A":
                    print(r_id+ " - go2goal")
                    time.sleep(0.15)
                    go2goal(start_pose, R_pose, WP_pub, status_pub, comp_task_pub)
              
            if R_task.pose.orientation.w == 0:
                print("no more tasks")
            rate.sleep()
        if check_update_task(R_task)==False:
            pass
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass