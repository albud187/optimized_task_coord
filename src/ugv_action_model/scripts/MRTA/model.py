import pandas as pd
import numpy as np

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