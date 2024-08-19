#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import time
import rosgraph
import rostopic
import pprint
import pandas as pd
#message imports
from geometry_msgs.msg import Pose, Vector3Stamped, Twist
from std_msgs.msg import String, Float64

#subscribed topics
T_POSEx = "/pose_est"
T_USGx = "/usg"
T_STATx = "/robot_status"
T_COMD = "/sys_cmd"

#published topics

#global variables
COMD_MSG = String()
ALL_POSE = {}
ALL_STATUS = {}
ALL_USG = {}
R = {}
#constants

#functions and classes
class robotAgent:
    def __init__(self, agent_id, x, y, action_cap):
        self.agent_id = agent_id
        self.x = x
        self.y = y
        self.action_cap = action_cap
        self.constraintUsage = 0
        self.allocUsage = 0
        self.status = "avb"

def list_topics():
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

def handle_pose(msg, args):
    global ALL_POSE
    arg1=args[0]
    #print(arg1)
    ALL_POSE[arg1]=msg

def handle_usg(msg, args):
    global ALL_USG
    arg1=args[0]
    #print(arg1)
    ALL_USG[arg1]=msg

def handle_status(msg, args):
    global ALL_STATUS
    arg1=args[0]
    #print(arg1)
    ALL_STATUS[arg1]=msg

def t_ns(topic):
    result = topic.split("/")[1]
    return result

def find_topics(topic_list, topic):
    result = []
    for t in topic_list:
        if topic in t:
            result.append(t)
    return result

all_topics = list_topics()

pose_topics = find_topics(all_topics, T_POSEx)
usg_topics = find_topics(all_topics, T_USGx)
stat_topics = find_topics(all_topics, T_STATx)
print(stat_topics)
agent_ids = []
for t in pose_topics:
    agent_ids.append("/"+t.split("/")[1])

def update_agents(ALL_POSE, ALL_USG, ALL_STATUS, R):
    for i in range(len(agent_ids)):
        r_id = agent_ids[i]
        pose_topic = r_id + T_POSEx
        usg_topic = r_id + T_USGx
        stat_topic = r_id + T_STATx

        R[r_id].x = ALL_POSE[pose_topic].position.x
        R[r_id].y = ALL_POSE[pose_topic].position.y
        R[r_id].constraintUsage = ALL_USG[usg_topic].data
        #print(ALL_STATUS)
        R[r_id].status = ALL_STATUS[stat_topic].data
    return R
def disp_objs(input_dict):
    for k in input_dict.keys():
        try:
            pprint.pprint(input_dict[k].__dict__)
        except:
            print(k)
            for t in input_dict[k]:
                pprint.pprint(t.__dict__)
            print(" ")
def update_df(df,R):
    for r in R.keys():
        xpos = round(R[r].x,2)
        ypos = round(R[r].y,2)
        milage = round(R[r].constraintUsage, 2)
        df.loc[r] = [xpos, ypos, milage, R[r].status]
    return df

def main_loop():
    global ALL_POSE
    global ALL_STATUS
    global ALL_USG
    global R
    rospy.init_node("system_monitor", anonymous = False)
    rate = rospy.Rate(5)
    #subscribers
    all_pose_topics = find_topics(list_topics(), "pose")
    
    
    for i in range(len(agent_ids)):
        r_id = agent_ids[i]
        
        pose_topic = pose_topics[i]
        usg_topic = usg_topics[i]
        stat_topic = stat_topics[i]

        ALL_POSE[pose_topic] = Pose()
        rospy.Subscriber(pose_topic, Pose, handle_pose,[pose_topic])

        ALL_USG[usg_topic] = Float64()
        rospy.Subscriber(usg_topic, Float64, handle_usg,[usg_topic])      
        
        ALL_STATUS[stat_topic] = String() 
        rospy.Subscriber(stat_topic, String, handle_status,[stat_topic])   

        r_pos_x = ALL_POSE[pose_topic].position.x
        r_pos_y = ALL_POSE[pose_topic].position.y

        R[r_id] = robotAgent(r_id, r_pos_x, r_pos_y, 1000)
    df = pd.DataFrame(columns = ["xpos", "ypos", "usage", "status",])
    while not rospy.is_shutdown():
        R = update_agents(ALL_POSE, ALL_USG, ALL_STATUS, R)
        DF = update_df(df, R)
        pprint.pprint(DF)
        print(" ")
        # pprint.pprint(ALL_POSE)
        # print("--")
        # print(ALL_STATUS)
        # print("--")
        # print(ALL_USG)
        # print(" ")
        rate.sleep()
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass