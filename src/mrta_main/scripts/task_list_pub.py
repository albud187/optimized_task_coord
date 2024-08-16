#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import time
import rosgraph
import rostopic
import pandas as pd

#message imports
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float64, Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Int64

#task allocation imports:
from MRTA.model import robotAgent

from MRTA.model import (csv_task, suitability_csv, 
                        robot_csv, plot_allocation)


from MRTA.greedy import offline_allocation
from MRTA.genetic_helper import (Af_to_sol, sol_to_Af, 
                                 suitability_matrix, prereq_and_deps, 
                                 full_list, cost_matrix2, 
                                 display_performance)

from MRTA.genetic import (generate_population, 
                          genetic_algorithm_iteration, 
                          fitness)

from MRTA._MRTA_constants import *

print("IMPORTS GOOD TO GO")
#subscribed topics
T_POSEx = "/pose_est"
T_USGx = "/usg"
T_STATx = "/robot_status"
T_COMD = "/sys_cmd"
T_WPx = "/WP"
T_task_listx = "/task_list"

#published topics
T_taskx = "/task"

#global variables
COMD_MSG = PoseStamped()
cmd_check =""
#dictionary of objects
ALL_POSE = {}
ALL_STATUS = {}
ALL_USG = {}
TL_PUBS = {}
TL_checks = {}
R = {}

R_av = {}
T = {}

REMAINING_T = {}
time.sleep(1)

robots_input = os.path.join(os.path.join(SCENARIO_DIR, SCENARIO_NAME), "agents.csv")
suitability_input = os.path.join(os.path.join(SCENARIO_DIR, SCENARIO_NAME), "suitabilities.csv")
tasks_input =  os.path.join(os.path.join(SCENARIO_DIR, SCENARIO_NAME), "tasks.csv")

#params
suitability_dict = suitability_csv(suitability_input)
robot_dict = robot_csv(robots_input)

#task allocation algorithm

#callbacks
def handle_alloc_task(msg):
    global T
    task_idx = msg.data
    T.pop(task_idx)

#dynamic insantiation
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

def find_topics(topic_list, subtopic):
    """
    inputs: list[str] topic_list, str subtopic
    output: list[str] result
    description: finds all topics that contain the subtopic
    """
    result = []
    for t in topic_list:
        if subtopic in t:
            result.append(t)
    return result

def update_agents(ALL_POSE, ALL_USG, ALL_STATUS):
    """
    inputs: dict ALL_POSE, ALL_USG, ALL_STATUS
    description: updates the state of all agents
    all agents described in R
    """
    global R
    for i in range(len(agent_ids)):
        r_id = agent_ids[i]
        pose_topic = r_id + T_POSEx
        usg_topic = r_id + T_USGx
        stat_topic = r_id + T_STATx
        # print(R.keys())
        # print(ALL_POSE.keys())
        R[r_id].x = ALL_POSE[pose_topic].position.x
        R[r_id].y = ALL_POSE[pose_topic].position.y
       
        R[r_id].constraintUsage = ALL_USG[usg_topic].data
        R[r_id].status = ALL_STATUS[stat_topic].data

def available_agents(R):
    """
    inputs: R dict[robotAgent] - dictionary of robotAgent objects describing state of robots
    outputs: R_avb dict[robotAgent] - dictionary of robotAgent objects describing state of robots but for robots that are available
    description: returns a dictionary of robots that are available
    """
    R_avb={}
    
    for r_id in R.keys():
        if R[r_id].status =="avb":

            R_avb[r_id] = R[r_id]
    return R_avb

def handle_pose(msg, args):
    """
    inputs: Pose msg, list[str] args
    outputs: None
    description: updates the dictionaries R[robotAgent] and R_av[RobotAgent] with information on robots' state
    arg is the name of the topic
    """
    global ALL_POSE
    global ALL_STATUS
    global ALL_USG
    global R
    global R_av
    arg1=args[0]
    #print(arg1)
    ALL_POSE[arg1]=msg
    update_agents(ALL_POSE, ALL_USG, ALL_STATUS)
    R_av=available_agents(R)

def handle_usg(msg, args):
    """
    inputs: Float64 msg, list[str] args
    outputs: None
    description: updates the dictionary ALL_USG[Float64] with usages of robots
    arg is the name of the topic
    """
    global ALL_USG
    arg1=args[0]
    #print(arg1)
    ALL_USG[arg1]=msg

def handle_status(msg, args):
    """
    inputs: String msg, list[str] args
    outputs: None
    description: updates the dictionary ALL_STATUS[Float64] with availabilities of robots
    arg is the name of the topic
    """
    global ALL_STATUS
    arg1=args[0]
    #print(arg1)
    ALL_STATUS[arg1]=msg

def greedy_genetic(A, n_iter, muta_prob, crossover_prob, 
                   adj, R0, s_mat, s_min, inter_task, T, pop_size):
    
    start_sol = Af_to_sol(A)[0]
    rkeys = Af_to_sol(A)[1]
    pop = generate_population(pop_size, start_sol, R0, T, 
                        suitability_dict, s_mat, s_min, inter_task)
    for _ in range(n_iter):
        pop = genetic_algorithm_iteration(muta_prob, crossover_prob, pop, adj, R0,
                                s_mat, suitability_dict, s_min, inter_task, T)

    fitnesses = []
    for p in pop:
        fitnesses.append(fitness(p, adj))
    best_fitness_idx = fitnesses.index(min(fitnesses))
    best_solution = pop[best_fitness_idx]
    
    new_allocation = sol_to_Af(best_solution, T, rkeys)
    return new_allocation

def generate_task_sequence(r_id, A, filepath):
    global TL_checks
    list_layout = MultiArrayLayout()
    list_dim = MultiArrayDimension()
    list_dim.label = filepath
    
    #check counter for list_dim.size
    list_dim.size = TL_checks[r_id]

    dimensions_list = [list_dim]
    list_layout.dim = dimensions_list

    result_list = Int16MultiArray()
    result_list.layout = list_layout
    
    task_seq=[]
    for t in A[r_id]:
        task_seq.append(t.idx)
    result_list.data = task_seq
    TL_checks[r_id] = TL_checks[r_id]+1
    return result_list


#get list of topics and all robots
all_topics = list_topics()
pose_topics = find_topics(all_topics, T_POSEx)
usg_topics = find_topics(all_topics, T_USGx)
stat_topics = find_topics(all_topics, T_STATx)
agent_ids = []
for t in pose_topics:
    agent_ids.append("/"+t.split("/")[1])

time.sleep(1)
def main_loop():

    global ALL_POSE
    global ALL_STATUS
    global ALL_USG
    global TL_PUBS
    global R
    global R_av
    global COMD_MSG
    global cmd_check

    rospy.init_node("task_list_publisher", anonymous = False)
    rate = rospy.Rate(4)
    scenario_name = rospy.get_param('~scenario_name', '31T-6R')
    #subscribers    
    #agent targetted subscribers and publishers
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
        
        R[r_id] = robotAgent(r_id, r_pos_x, r_pos_y, robot_dict[r_id[1:]]["type"], robot_dict[r_id[1:]]["actionCap"])
        
        TL_PUBS[r_id] = rospy.Publisher(r_id+T_task_listx, Int16MultiArray, queue_size=10)
        TL_checks[r_id]=1
    s_min = 0.01
    n_iter, pop_size, muta_prob, crossover_prob = 300, 10, 0.7, 0.7
    check_deps, check_prereqs, check_perf_idx = True, True, True
    Kd, Kt = 1,1
    plotSize = 7
    #loop
  
    
    #print("waiting")
    
    task_file =  os.path.join(os.path.join(SCENARIO_DIR, scenario_name), "tasks.csv")
    T = csv_task(task_file)
    
    nodes = full_list(R,T)
    adj = cost_matrix2(nodes)

    #greedy algorithm
    A,Rf = offline_allocation(R, T, check_deps, check_prereqs, check_perf_idx, 
                                s_min, Kd, Kt, suitability_dict)
    print("greedy algorithm stats")
    display_performance(A,adj)
    plot_allocation(R, A, T, plotSize)
    print(" ")
    # #genetic algorithm
    # print("executing genetic algorithm")

    # inter_task = prereq_and_deps(T)
    # rkeys = Af_to_sol(A)[1]
    # s_mat = suitability_matrix(suitability_dict, R, T, rkeys)
    # A2 = greedy_genetic(A, n_iter, muta_prob, crossover_prob, 
    #     adj, R, s_mat, s_min, inter_task, T, pop_size)
    # print("genetic algorithm stats")
    
    # display_performance(A2,adj)
    # plot_allocation(R, A2, T, plotSize)
    # A = A2
    
    print("executing")
    print(A.keys())
    AR = {}
    for r_i in list(A.keys()):
        AR[r_i]= generate_task_sequence(r_i, A, task_file)
        TS_i = generate_task_sequence(r_i, A, task_file)

        print(TL_PUBS.keys())
        
        TL_PUBS[r_i].publish(TS_i)
    time.sleep(3)
    

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass