import numpy as np
import copy

def cost_func_ij(robotAgent, task, Kd, Kt, check_deps, T, suitability_dict):
    distance = np.sqrt((robotAgent.x - task.x)**2+(robotAgent.y - task.y)**2)
    suitability = suitability_dict[robotAgent.agent_type][task.taskType]
    c_ij = Kd*distance + Kt*task.taskCost
    if check_deps == False:
        result = c_ij
        return (result, suitability)
    if check_deps == True:
        dep_ratio = dependancy_ratio(task,T)
        result = dep_ratio*c_ij
    return (result, suitability, dep_ratio)

class taskDetail():
    def __init__(self, robotAgent, task, Kd, Kt, check_deps, T, suitability_dict):
        self.assignedTo = robotAgent.agent_id
        self.idx = task.idx
        self.allocCost = cost_func_ij(robotAgent, task, Kd, Kt, check_deps, T, suitability_dict)[0]
        self.constraintCost = cost_func_ij(robotAgent, task, Kd, Kt, False, T, suitability_dict)[0]
        self.suitabilityScore = cost_func_ij(robotAgent, task, Kd, Kt, False, T, suitability_dict)[1]
        try:
            self.depRatio = cost_func_ij(robotAgent, task, Kd, Kt, check_deps, T, suitability_dict)[2]
        except:
            pass
        self.prereqs = task.prereqs
        self.allocUsage = robotAgent.allocUsage + self.allocCost
        self.constraintUsage = robotAgent.constraintUsage + self.constraintCost
        self.action_cap = robotAgent.action_cap


def dependancy_ratio(T_j, T):
    
    #dependants is a dictioanry with key = task index and entry = distance to depednant
    #non_deps = 
    dependants = {}
    non_deps = {}
    for i in T.keys():
        t = T[i]
        
        if set([T_j.idx]).issubset(t.prereqs):
            distance2dependant = np.sqrt((T_j.x-t.x)**2+(T_j.y-t.y)**2)
            dependants[t.idx] = distance2dependant
        else:
            distance2nondep = np.sqrt((T_j.x-t.x)**2+(T_j.y-t.y)**2)
            non_deps[t.idx] = distance2nondep
    n_dependants = len(dependants.keys())
    n_nondeps = len(non_deps.keys())
    
    if n_dependants == 0:
        result = 1
        return result
    if len(T) == n_dependants:
        result = 0
        return result
    else:
    
        dep_ratio = n_nondeps/(n_nondeps + n_dependants)

        dist_deps = 0
        dist_nondeps = 0

        for t in dependants.keys():
            dist_deps += dependants[t]
        for t in non_deps.keys():
            dist_nondeps += non_deps[t]

        avg_dist_dep = dist_deps/n_dependants
        avg_dist_nondeps = dist_nondeps/n_nondeps
        
    result = 1-(n_dependants/(n_nondeps+n_dependants))*(1-avg_dist_dep/(avg_dist_dep+avg_dist_nondeps))
    return result
    
def TD_set(R,T,Kd,Kt, check_deps, suitability_dict):
    """
    inputs: str filepath must be csv file
    output: dict[dict[float]] suitability_dict
    description: takes csv file for agent-task sutiability and returns it as a dictionary
    """
    TD = {}
    for r in R.keys():
        TD[r] = []
        for i in T.keys():
            t = T[i]
            T_ij = taskDetail(R[r],t,Kd, Kt, check_deps, T, suitability_dict)
            TD[r].append(T_ij)
    return TD

def single_filter(T_ij, T_a, s_min, filter_prereqs_check):
    """
    inputs: taskDetail T_ij, RobotAgent r, float s_min, list T_a, bool check_prereqs
    result: bool
    descriptions: determines if a task is admissiable or not
    inadmissible means prereqs not met, tasktype not matching payload type, cost exceeds available range
    """
    if filter_prereqs_check == True:
        if set(T_ij.prereqs).issubset(T_a) == False:
            return False
        if T_ij.suitabilityScore < s_min:
            return False
        if T_ij.constraintUsage > T_ij.action_cap:
            return False
        else:
            return True
    else:
        if T_ij.suitabilityScore < s_min:
            return False
        if T_ij.constraintUsage > T_ij.action_cap:
            return False
        else:
            return True
#filtered_CT -->TD_filter   
def TD_filter(TD, T_a, s_min, filter_prereqs_check):
    """
    inputs: dict[RobotAgengt] R, dict[taskDetail] TD, list[int] Ta
    output: dict[taskDetail] TD
    description: returns set of admissisable agent-task matchings
    admissible means same type, does not break constraints, prereqs met
    """
    TDf = {}
    for r in TD.keys():
        TDf[r] = []
        for T_ij in TD[r]:
            if single_filter(T_ij, T_a, s_min, filter_prereqs_check) == True:
                TDf[r].append(T_ij)
    return TDf

def allocate(CT_f, check_perf_idx):
    """
    inputs: dict[taskDetail] TDf
    outputs: taskDetail tij
    description: determines most locally optimal allocation
    """
    CT_min = []
    for r in CT_f.keys():
        try:
            CT_min_r = min(CT_f[r], key=lambda x: x.allocCost)
            CT_min.append(CT_min_r)
        except:
            pass
    #enforce performance index
    if check_perf_idx == True:
        Tij = min(CT_min, key=lambda x: x.constraintUsage)
        
    else:
        Tij = min(CT_min, key=lambda x: x.allocCost)
        
    
    return Tij

def update_agents(R, A, T_ij):
    
    r = T_ij.assignedTo
    task_idx = T_ij.idx
    
    #update position
    
    last_task = A[r][-1]
    R[r].x = last_task.x
    R[r].y = last_task.y
    R[r].allocUsage = T_ij.allocUsage
    R[r].constraintUsage = T_ij.constraintUsage
    return(R)


def performance_index(numbers):
    n = len(numbers)
    numbers.sort()
    coef = 2 / n
    const = (n + 1) / n
    weighted_sum = sum([(i+1) * numbers[i] for i in range(n)])
    gini_coeff = (coef * weighted_sum / (np.sum(numbers)) - const)
    result = 1 - gini_coeff
    return result

def greedy_allocation(R0, T0, use_dep_ratio, check_prereqs, check_perf_idx, s_min, Kd, Kt, suitability_dict):
    """
    inputs: dict[robotAgent] R0, dict[task] T0
    result: tuple( dict[list[task]] A, dict[robotAgent] R )
    description: allocates tasks to robots, returns allocation and sequence of tasks as A
    """
    R = copy.deepcopy(R0)
    T = copy.deepcopy(T0)
    n_tasks = len(T0)
    Ta = []
    A={}
    for r in R.keys():
        A[r] = []
    for j in range(n_tasks):
       
        #allocate
        try:
            TD = TD_set(R, T, Kd, Kt, use_dep_ratio, suitability_dict)
            TDf = TD_filter(TD, Ta, s_min, check_prereqs)
            T_ij = allocate(TDf, check_perf_idx)
            
            r_id, t_idx = T_ij.assignedTo, T_ij.idx
            A[r_id].append(T[t_idx])
            Ta.append(t_idx)

            #update
            T.pop(t_idx)
            R = update_agents(R,A,T_ij)
        except:
            pass
    
    return (A,R,T)