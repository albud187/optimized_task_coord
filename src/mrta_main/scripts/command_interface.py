#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import time
import rosgraph
import rostopic
#message imports
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64

#subscribed topics

#published topics
T_COMD = "/sys_cmd"
T_WPx = "/WP"
#global variables
COMD_MSG = PoseStamped()

#constants

def list_topics():
    """
    returns list of all topics
    """
    master = rosgraph.Master('/rostopic')
    pubs, subs = rostopic.get_topic_list(master=master)
    topic_data = {}
    
    for topic in pubs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
        topic_data[name]['publishers'] = topic[2]
    
    for topic in subs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
        topic_data[name]['subscribers'] = topic[2]

    return sorted(topic_data.keys())

def find_topics(topic_list, s_t):
    """
    inputs: list of all topics, string s_t
    output: list of all topics with string s_t in it
    """
    result = []
    for t in topic_list:
        if s_t in t:
            result.append(t)
    return result

#find all namespaces
all_topics = list_topics()
target_topic = "pose_est"
pose_topics = find_topics(all_topics, target_topic)
agent_ids = []
for t in pose_topics:
    agent_ids.append(t.split("/")[1])

print(agent_ids)

#functions


def subpub_syscomd(msg):
    """
    publishes COMD_MSG each callback loop
    """
    global COMD_MSG
    rate = rospy.Rate(10)
    cmd_pub = rospy.Publisher(T_COMD, PoseStamped, queue_size=10)
    cmd_pub.publish(COMD_MSG)
    rate.sleep()

def validate_command(com_in):
    """
    input:string com_in
    checks command formatting, returns True if command is valid and False if not
    n=s (sets agent n to status available, makes it stop in place)
    n=b (sets agent n to status broken)
    n=x,y (assigns agent n to waypoint x,y)
    a=filename.csv (assigns tasks from filename.csv optimally)
    a=s (stops all agents, sets to status available)
    a=x,y (optimally assigns an agent to waypoint x,y)
    
    """
    try:
        comd_list = com_in.split("=")
        try: 
            robot_idx = int(comd_list[0])
            print(robot_idx)
            if comd_list[1] == "s":
                return True
            if comd_list[1] == "b":
                return True
            else:
                try:
                    WP = comd_list[1].split(",")
                    WP[0], WP[1] = float(WP[0]), float(WP[1])
                    print(WP)
                    return True
                except:
                    return False
        except:
            if comd_list[0] == "a":
                if comd_list[1].endswith(".csv"):
                    return True
                if comd_list[1] =="s":
                    return True
                else:
                    try:
                        WP = comd_list[1].split(",")
                        WP[0], WP[1] = float(WP[0]), float(WP[1])
                        return True
                    except:
                        return False
    except:
        return False



def main_loop():
    #init
    global COMD_MSG
    rospy.init_node("command_interface", anonymous = False)
    rate = rospy.Rate(30)
    
    #subscribers
    
    
    #publishers
    cmd_pub = rospy.Publisher(T_COMD, PoseStamped, queue_size=10)

    #loop
    #rospy.spinOnce()
    user_input = input("enter command: ")
    if validate_command(user_input):
        COMD_MSG.header.frame_id = user_input
        cmd_pub.publish(COMD_MSG)
    else:
        print("invalid command")
    while not rospy.is_shutdown():
        
        #rospy.Subscriber(T_COMD, String, subpub_syscomd)
        user_input = input("enter command: ")
        if validate_command(user_input):
            COMD_MSG.header.frame_id  = user_input
            cmd_pub.publish(COMD_MSG)
        else:
            print("invalid command")
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
    