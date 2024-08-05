#helper functions to go from greedy allocation to hill climbing / genetic algorithm
import numpy as np
from MRTA.model import node

def full_list(R0,T):
    n_robot = len(R0)
    n_task = len(T)
    result = []
    count = 0
    for r in R0.keys():
        result.append(node(count, R0[r].x, R0[r].y, r,0))
        count = count + 1
    t_n = n_robot
    for t in T.keys():
        result.append(node(t_n,T[t].x, T[t].y,"task",T[t].taskCost))
        t_n = t_n + 1
         
    return result

def calculate_route_cost(soln, adj):
    s = 0
    for i in range(1,len(soln)):
        idx1 = soln[i-1]
        idx2 = soln[i]
        s+= adj[idx1,idx2]
    return s

def fitness(chromo, adj):
    path_lengths = []
    for path in chromo:
        path_lengths.append(calculate_route_cost(path, adj))
    result = sum(path_lengths)+max(path_lengths)
    return result

def distance_func(n1, n2):
    result = np.sqrt((n1.x-n2.x)**2+(n1.y-n2.y)**2)
    return result

def cost_matrix2(N):
    dim_CM = len(N)
    cost_matrix = np.zeros((dim_CM,dim_CM))
    for i in range(len(N)):
        for k in range(len(N)):
            if i !=k:
                cost_matrix[i][k] = round(distance_func(N[i],N[k])+N[k].taskCost,3)
    return (cost_matrix)

def Af_to_sol(Af):
    result = []
    n_robots = len(Af)
    robots = list(Af.keys())
    for r in range(len(robots)):
        TL = [r]
        r_id = robots[r]
        for t in Af[r_id]:
            TL.append(t.idx+len(Af))
        result.append(np.array(TL))
    return result,robots


def sol_to_Af(soln, T, rkeys):
    result = {}
    n_robots = len(rkeys)
    for i in range(n_robots):
        r_id = rkeys[i]
        result[r_id] = []
        for t in soln[i][1:]:
            result[r_id].append(T[t-n_robots])
    return result

def suitability_matrix(suitability_dict, R, T0, rkeys):
    result = []
    n_robots = len(rkeys)
    for r in rkeys:
        #R_suitability = n_robots*[-1]
        R_suitability = []
        r_type = R[r].agent_type
        for t_idx in list(T0.keys()):
            t = T0[t_idx]
            task_type = t.taskType
            suitability = suitability_dict[r_type][task_type]
            R_suitability.append(suitability)
        result.append(np.array(R_suitability))
    
    return result

def prereq_list(T):
    result = []
    for i in sorted(list(T.keys())):
        result.append(T[i].prereqs)
    return result
        
def find_num_simple(new_arr,num):
    num_loc = np.where(new_arr==num)
    result = num_loc[0][0], num_loc[1][0]
    return result

def find_num(arr_list, num):
    dim = max([len(a) for a in arr_list])
    new_arr = np.full((len(arr_list), dim), -99)
    for i, arr in enumerate(arr_list):
        new_arr[i, :len(arr)] = arr
    
    num_loc = np.where(new_arr==num)
    result = num_loc[0][0], num_loc[1][0]
    return result

def prereq_and_deps(T):
    result = {}
    for t in T:
        result[t] = {"idx": T[t].idx, "deps": [], "prereqs": T[t].prereqs}
    for t, task in result.items():
        for prereq in task["prereqs"]:
            result[prereq]["deps"].append(task["idx"])
    return result

def performance_index(numbers):
    n = len(numbers)
    numbers.sort()
    coef = 2 / n
    const = (n + 1) / n
    weighted_sum = sum([(i+1) * numbers[i] for i in range(n)])
    gini_coeff = (coef * weighted_sum / (np.sum(numbers)) - const)
    result = 1 - gini_coeff
    return result

def display_performance(A, adj):
    sol = Af_to_sol(A)
    routes = sol[0]
    rkeys = sol[1]
    costs = []
    for path in routes:
        rotue_cost = calculate_route_cost(path,adj)
        costs.append(rotue_cost)
    print("robots: "+str(rkeys))
    print("costs "+str(costs))
    print("total cost: "+str(sum(costs)))
    print("performance index: "+str(performance_index(costs)))
    print("minmax cost :"+str(sum(costs)+max(costs)))