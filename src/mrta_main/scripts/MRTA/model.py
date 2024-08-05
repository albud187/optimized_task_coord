import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


class node:
    def __init__(self, idx, x, y, ntype, taskCost):
        self.idx = int(idx)
        self.x = x
        self.y = y
        self.type = ntype
        self.taskCost = taskCost

class task:
    def __init__(self, idx, x, y, taskType, taskCost, prereqs):
        self.idx = idx
        self.x = x
        self.y = y
        self.taskType = taskType
        self.taskCost = taskCost
        self.prereqs = prereqs

class robotAgent:
    def __init__(self, agent_id, x, y, agent_type, action_cap):
        self.agent_id = agent_id
        self.x = x
        self.y = y
        self.agent_type = agent_type
        self.action_cap = action_cap
        self.constraintUsage = 0
        self.allocUsage = 0

def robot_csv(filename):
    """
    inputs: str filename (csv)
    outputs:dict[dict] result
    description: takes a csv file and returns dictionary with robots type, payload count, range
    """
    df = pd.read_csv(filename)
    result_dict = {}
    for i in range(len(df)):
        df_row = df.iloc[i]
        ns = df_row.ns
        result_dict[ns] = {"type":df_row.robotType, "actionCap":float(df_row.actionCap)}
    return result_dict

def csv_task(filename):
    """
    input:string filename must be a csv file
    output: dict [task] taskDict
    description: converts csv file into dataframe into dictonary of tasks
    this is our "list" of tasks to be optimally allocated
    """
    df = pd.read_csv(filename, index_col = 0)
    TaskDict = {}
    
    for i in range(len(df)):
        t = df.iloc[i]
        
        list_prereq = t.prereqs[1:-1].split(",")
    
        try:
            for k in range(len(list_prereq)):
                list_prereq[k]=int(list_prereq[k])
        except:
            list_prereq = []
        
        TaskDict[t.idx] = task(t.idx, t.x, t.y, t.taskType, t.taskCost, list_prereq)

    return TaskDict

def suitability_csv(filepath):
    """
    inputs: str filepath must be csv file
    output: dict[dict[float]] suitability_dict
    description: takes csv file for agent-task sutiability and returns it as a dictionary
    """
    
    df = pd.read_csv(filepath)
    suitability_dict = {}
    for i in range(len(df)):
        r_type = df.iloc[i].robot_type
        suitability_dict[r_type] = {}
        for t_type in list(df.iloc[i].keys())[1:]:
            suitability_dict[r_type][t_type]=df.iloc[i][t_type]
    return suitability_dict

def plot_allocation(R0, A, T0, size):
    fig = plt.figure(figsize=(size, size))
    colors = ["gray", "red", "blue", "violet", "tan", "purple"]
    robots_list = list(R0.keys())
    all_x_coords = []
    all_y_coords = []
    for t in T0.keys():
        plt.scatter(T0[t].x, T0[t].y, linewidths=0, c="black")
        task_idx = str(T0[t].idx)
        plt.text(T0[t].x+0.05, T0[t].y+0.05, task_idx, c="black")
        all_x_coords.append(T0[t].x)
        all_y_coords.append(T0[t].y)
    
    for i in range(len(robots_list )):     
        r = robots_list[i]
        all_x_coords.append(R0[r].x)
        all_y_coords.append(R0[r].y)
        color = colors[i]
        
        start = (R0[r].x, R0[r].y)
        coords = []
        for at in A[r]:
            coords.append([at.x, at.y])
        
        df_dict = {"x":[start[0]], "y":[start[1]]}
        plt.scatter(start[0], start[1], linewidths=8, c=color)
        #plot_text = r +" : " + str(round(R[r].constraintUsage,2)) +" / " +str(R[r].action_cap)
        #plot_text = r +" : " + str(round(R[r].constraintUsage,2))
        plot_text = r
        plt.text(start[0]-0.8, start[1]+0.5, plot_text, c=color)
        #annotate
        for coord in coords:
            df_dict["x"].append(coord[0])
            df_dict["y"].append(coord[1])
        df = pd.DataFrame.from_dict(df_dict)
#         to_plot = coords
#         start = (R0[r].x, R0[r].y)

        for i,row in df.iterrows():
            if i==0:
                pass
            else:
                
                plt.annotate('',xy=(row['x'],row['y']),xytext=(df.iloc[i-1]['x'],df.iloc[i-1]['y']),
                arrowprops=dict(color=color,width=0.3,headwidth=4))
    
    xmin = min(all_x_coords) - 0.05*(max(all_x_coords)-min(all_x_coords)-1)
    xmax = max(all_x_coords) + 0.05*(max(all_x_coords)-min(all_x_coords)+1)
    ymin = min(all_y_coords) - 0.05*(max(all_y_coords)-min(all_y_coords)-1)
    ymax = max(all_y_coords) + 0.05*(max(all_y_coords)-min(all_y_coords)+1) 
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.show()