#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
import os
import time
#message imports
from std_msgs.msg import Int16MultiArray, String, Int16
from geometry_msgs.msg import PoseStamped
from MRTA.model import csv_task, task, robotAgent

r_id = rospy.get_namespace()
#subscribed topics
T_task_list = r_id + "/task_list"
T_stat = r_id + "/robot_status"

#published tasks
T_task = r_id + "/task"
T_alloc_task = r_id+"/alloc_task"
#globals
task_list = Int16MultiArray()
R_status = String()
R_task = PoseStamped()
Ti=[]
#callbacks
def handle_task_list(msg):
    global task_list
    global Ti
    task_list = msg
    filename = msg.layout.dim[0].label
    all_tasks = csv_task(filename)
    Ti = ri_task_sequence(task_list.data, all_tasks)
    print(Ti)

def handle_status(msg):

    global R_status
    R_status = msg

#functions
prev_list_seq = 0
def check_new_list(task_list):
    global prev_list_seq
    try:
        task_list.layout.dim[0].size
        if task_list.layout.dim[0] != prev_list_seq:
            result = True
        else:
               result = False
        prev_list_seq = task_list.layout.dim[0]
        return result
    except:
        return False
def task_param(Tj):
    """
    inputs: task Tj
    outputs: PoseStamped task_description
    description: converts a task into representation using PoseStamped datatype
    px, py = task location (x, y)
    pz,rx,ry  = param
    rz = tasktype
    rw = status
    """
    task_desc = PoseStamped()

    task_desc.header.frame_id = Tj.taskType
    task_desc.pose.position.x = Tj.x
    task_desc.pose.position.y = Tj.y
    task_desc.pose.orientation.w = 1
    task_desc.pose.orientation.x = Tj.taskCost

    return task_desc

#functions
def ri_task_sequence(task_list, all_tasks):
    result = []
    for ti in task_list:
        result.append(all_tasks[ti])
    return result

def main_loop():
    global task_list
    global Ti
    rospy.init_node("task_list_sub", anonymous=False)
    rate = rospy.Rate(10)

    #subscribers
    rospy.Subscriber(T_task_list, Int16MultiArray ,handle_task_list)
    rospy.Subscriber(T_stat, String, handle_status)
   
    #publishers
    task_pub = rospy.Publisher(T_task, PoseStamped, queue_size=10)
    comp_task_pub = rospy.Publisher(T_alloc_task, Int16, queue_size=10)
    while not rospy.is_shutdown():
        #if tasks are recieved
        while len(Ti)>0:
            time.sleep(0.4)
            if R_status.data == "avb":
                print(r_id+" - "+ str(Ti[0].__dict__))
                comp_task_idx = Int16()
                comp_task_idx.data = Ti[0].idx
                comp_task_pub.publish(comp_task_idx)
                task_details = task_param(Ti[0])
                task_pub.publish(task_details)
                Ti.pop(0)
                time.sleep(0.4)
        #print("no more tasks")
            
        rate.sleep()
    

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass