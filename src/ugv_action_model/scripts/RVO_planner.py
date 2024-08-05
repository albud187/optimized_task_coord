#!/usr/bin/env python3

#package imports
import rospy, rostopic, rosgraph
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
import pprint
#message imports
from geometry_msgs.msg import Twist, Vector3Stamped, Pose2D, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

TB3_ID = rospy.get_namespace()

#subscribed topics
#T_POSEx = "/pose_odom"

T_POSEx = "/pose_est"
T_VELax = "/vel_a"
T_WPx = "/WP"
T_VELdx = "/vel_d"
#published topics

T_VELd = TB3_ID+"vel_d"

#global variables
ALL_POS = {}
ALL_V_d = {}
ALL_V_a = {}
ALL_WP = {}
VEL_D_PUBS = {}
VMAX = 0.26
POS_TOL = 0.05
ROBOT_RADIUS = 0.3
#constants
time.sleep(1)
#callbacks
def handle_pose(msg, args):
    global ALL_POS
    arg1=args[0]
    ALL_POS[arg1]=[msg.position.x, msg.position.y]
    
def handle_vd(msg, args):
    global ALL_V_d
    arg1=args[0]
    ALL_V_d[arg1]=[msg.vector.x, msg.vector.y]

def handle_va(msg, args):
    global ALL_V_a
    arg1=args[0]
    ALL_V_a[arg1]=[msg.vector.x, msg.vector.y]

def handle_wp(msg, args):
    global ALL_WP
    arg1=args[0]
    ALL_WP[arg1]=[msg.vector.x, msg.vector.y]

#functions
def list_topics():
    """
    output: list of all topics
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

def find_topics(topic_list, topic):
    result = []
    for t in topic_list:
        if topic in t:
            result.append(t)
    return result

#get topics get all robots


all_topics = list_topics()
pose_topics = find_topics(all_topics, T_POSEx)
agent_ids = []
for t in pose_topics:
    agent_ids.append("/"+t.split("/")[1])

vel_act_topics = find_topics(all_topics, T_VELax)
while len(vel_act_topics)<len(agent_ids)-1:
    vel_act_topics = find_topics(all_topics, T_VELax)


vel_d_topics = find_topics(all_topics, T_VELdx)
wp_topics =  find_topics(all_topics, T_WPx)

def list_V_max(n_robots, vmax):
    result = []
    for i in range(n_robots):
        result.append(vmax)
    return result

def ALL_POS_X(ALL_POS):
    """
    input: dict ALL_POS
    output: list X
    result: takes in positions of all robots, returns dict of [x,y] values for their positions
    """
    X = {}
    for t_pos in ALL_POS.keys():
        r_id = t_pos.split("/")[1]
        pos = ALL_POS[t_pos]
        X[r_id] = pos
    return X

def WP_GOAL(ALL_WP, X, agent_ids):
    """
    input: dict ALL_WP, list X, list agent_ids
    output: list GOALS
    result: takes in goals of all robots, returns list of [x,y] values for their goals
    """
    GOALS = {}
    for i in range(len(agent_ids)):
        r_id=agent_ids[i]
        T_WP = r_id + T_WPx
        try:
            GOALS[r_id[1:]]=(ALL_WP[T_WP])
        except:
            GOALS[r_id[1:]]=X[r_id[1:]]
    return GOALS

def get_V_curr(ALL_V_a, agent_ids):
    """
    
    """
    V_curr = {}
    for i in range(len(agent_ids)):
        r_id = agent_ids[i]
        T_VELa = r_id + T_VELax
        V_curr[r_id[1:]] =ALL_V_a[T_VELa]
        
    return V_curr

def compute_V_des(X, goal, V_max):
    V_des = {}
    
    for i in X.keys():


        dif_x = [goal[i][0]-X[i][0], goal[i][1]-X[i][1]]
        #norm is magntiude of dif_x
        norm = distance(dif_x, [0, 0])
        uv = [dif_x[k]/norm for k in range(2)]
        if norm > V_max:
            k = V_max
        else:
            k = norm
        v_i = [k*uv[j] for j in range(2)]
        
        V_des[i] = (v_i[:])
        if reach(X[i], goal[i], POS_TOL):
            V_des[i]= [0,0]
            
    return V_des

#RVO Algorithm
def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)+0.001

def reach(p1, p2, bound):
    if distance(p1,p2)< bound:
        return True
    else:
        return False

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= np.pi:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*np.pi
            if theta_dif < 0:
                theta_dif += 2*np.pi
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*np.pi
            if theta_dif < 0:
                theta_dif += 2*np.pi
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def intersect(pA, vA, RVO_BA_all):

    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []
    for theta in np.arange(0, 2*np.pi, 0.1):
        for rad in np.arange(0.02, norm_v+0.02, norm_v/5.0):
            new_v = [rad*np.cos(theta), rad*np.sin(theta)]
            suit = True
            for i in RVO_BA_all.keys():
                RVO_BA = RVO_BA_all[i]
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
               
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = np.arctan2(dif[1], dif[0])
                theta_right = np.arctan2(right[1], right[0])
                theta_left = np.arctan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)                
    new_v = vA[:]
    suit = True
    for i in RVO_BA_all.keys():
        RVO_BA = RVO_BA_all[i]
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = np.arctan2(dif[1], dif[0])
        theta_right = np.arctan2(right[1], right[0])
        theta_left = np.arctan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    #----------------------        
    if suitable_V:
        
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
        new_v = vA_post[:]
        for i in RVO_BA_all.keys():
            RVO_BA = RVO_BA_all[i]
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = np.arctan2(dif[1], dif[0])
            theta_right = np.arctan2(right[1], right[0])
            theta_left = np.arctan2(left[1], left[0])
    else:
        
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for i in RVO_BA_all.keys():
                RVO_BA = RVO_BA_all[i]
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = np.arctan2(dif[1], dif[0])
                theta_right = np.arctan2(right[1], right[0])
                theta_left = np.arctan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*np.sin(small_theta)) >= rad:
                        rad = abs(dist*np.sin(small_theta))
                    big_theta = np.arcsin(abs(dist*np.sin(small_theta))/rad)
                    dist_tg = abs(dist*np.cos(small_theta))-abs(rad*np.cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/distance(dif, [0,0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA)))
    return vA_post

def RVO_update(X, V_des, V_current):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    #inflate robot radius
    ROB_RAD = ROBOT_RADIUS
    
    #set optimal velocity as current velocity
    V_opt = V_current
    
    #iterate thru all robots positions
    for i in X.keys():
        
        #vA and pA are current robot's velocity and position
        vA = V_current[i]
        pA = X[i]
        RVO_BA_all = {}
        #calculates the velocity obstacles of robot j with respect to robot i
        for j in X.keys():
            if i!=j:
                #calculate other robots velocity obstacles
                vB = V_current[j]
                pB = X[j]
              
                transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
                
                dist_BA = distance(pA, pB)
                theta_BA = np.arctan2(pB[1]-pA[1], pB[0]-pA[0])
                if 2*ROB_RAD > dist_BA:
                    dist_BA = 2*ROB_RAD
                theta_BAort = np.arcsin(2*ROB_RAD/dist_BA)
                theta_ort_left = theta_BA+theta_BAort
                
                bound_left = [np.cos(theta_ort_left), np.sin(theta_ort_left)]
                theta_ort_right = theta_BA-theta_BAort
                bound_right = [np.cos(theta_ort_right), np.sin(theta_ort_right)]
                
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                RVO_BA_all[j] = RVO_BA            

        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
        
    return V_opt
time.sleep(1)
#main loop
def main_loop():
    global ALL_POS
    global ALL_V_d
    global ALL_V_a
    global ALL_WP
    global agent_ids
    rospy.init_node("RVO_planner", anonymous = False)
    rate = rospy.Rate(40)
    time.sleep(1)
    #agent targetted subscribers and publishers
    for i in range(len(agent_ids)):
        r_id = agent_ids[i]
        
        pose_topic = pose_topics[i]
        vel_d_topic = vel_d_topics[i]
        vel_act_topic = vel_act_topics[i]
        wp_topic = wp_topics[i]
        rospy.Subscriber(pose_topic, Pose, handle_pose,[pose_topic])
        rospy.Subscriber(vel_d_topic , Vector3Stamped, handle_vd, [vel_d_topic])
        rospy.Subscriber(vel_act_topic, Vector3Stamped, handle_va, [vel_act_topic])
        rospy.Subscriber(wp_topic, Vector3Stamped, handle_wp, [wp_topic])
        
        T_VELd = r_id + T_VELdx
        VEL_D_PUBS[r_id] = rospy.Publisher(T_VELd , Vector3Stamped, queue_size=10)


    while not rospy.is_shutdown():
        try:
            X = ALL_POS_X(ALL_POS)

            GOALS = WP_GOAL(ALL_WP, X, agent_ids)
          
            V_des = compute_V_des(X, GOALS, VMAX)
            
            V_curr = get_V_curr(ALL_V_a, agent_ids)

            V = RVO_update(X, V_des, V_curr)
          
            for i in V.keys():

                r_id = "/"+ i 
                vel_d = Vector3Stamped()
                vel_d.vector.x = V[i][0]
                vel_d.vector.y = V[i][1]
                VEL_D_PUBS[r_id].publish(vel_d)
            print(V)
        except Exception as e:
             print(e)

        rate.sleep()
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass