import numpy as np
import random
import copy

from MRTA.genetic_helper import Af_to_sol

def find_num_simple(new_arr,num):
    num_loc = np.where(new_arr==num)
    result = num_loc[0][0], num_loc[1][0]
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

def to_TA(soln):
    n_robots = len(soln)
    assigned_tasks_by_idx = [route[1:]-n_robots for route in soln]
    return assigned_tasks_by_idx

def generate_random_alloc(R0, T, suitability_dict, s_min):
    A = {r: [] for r in R0.keys()}
    T_a = set()
    n_robots = len(R0)
    robots = list(R0.keys())
    #random.shuffle(robots)
    tasks = list(T.values())
    random.shuffle(tasks)
   
    while len(tasks)>0:
        for t in tasks:
            assignable_robots = [r for r in robots if suitability_dict[R0[r].agent_type][t.taskType] >= s_min and t.idx not in T_a and set(t.prereqs).issubset(T_a)]
            if assignable_robots:
                assigned_robot = random.choice(assignable_robots)
                A[assigned_robot].append(t)
                T_a.add(t.idx)
                tasks.pop(tasks.index(t))
    soln = Af_to_sol(A)[0]
    return soln

def generate_population(pop_size, start_sol, R0, T, 
                        suitability_dict, suitability_mat, s_min, inter_task):
    
    population = [start_sol]
    random_pop_size = int(pop_size*0.6)
    remainder_size = pop_size - random_pop_size - 1
    
    for i in range(random_pop_size):
        new_chromo = generate_random_alloc(R0, T, suitability_dict, s_min)
        population.append(new_chromo)
    for i in range(remainder_size):
        mod_chromo = copy.deepcopy(start_sol)
        mod_chromo = take_mutate2(mod_chromo, suitability_mat, s_min, inter_task)
        population.append(mod_chromo)
    return population


def task_range(inter_task, arr_list, tj):
    dim = max([len(a) for a in arr_list])
    new_arr = np.full((len(arr_list), dim), -99)
    for i, arr in enumerate(arr_list):
        new_arr[i, :len(arr)] = arr
    
    task_rel = inter_task[tj]
    deps = task_rel["deps"]
    prereqs = task_rel["prereqs"]
    
    lower_bounds = []
    upper_bounds = []
    for t1 in prereqs:
        lower_bounds.append(find_num_simple(new_arr, t1)[1])
    for t2 in deps:
        upper_bounds.append(find_num_simple(new_arr, t2)[1])
    
    try:
        min_range = max(lower_bounds)
    except:
        min_range = 0
    
    try:
        max_range = min(upper_bounds)
    except:
        chosen_robot = find_num_simple(new_arr, tj)[0]
        max_range = len(arr_list[chosen_robot])
    #print(new_arr)
    return min_range, max_range


def take_mutate2(soln, suitability_mat, s_min, inter_task):
    """
    inputs: list[array] chromo
    outputs: list[array] result
    description: takes in a solution set, selects 2 routes between them, takes one node from one route 
    assigns it to another route at random
    """
    n_robots = len(soln)
    assigned_tasks_by_idx = [route[1:]-n_robots for route in soln]
    r1 = np.random.randint(0,len(soln))

    while len(assigned_tasks_by_idx[r1])<=2:
        r1 = np.random.randint(0,len(soln))
    tj = random.choice(assigned_tasks_by_idx[r1])
    
 
    viable_robots = []
    for i in range(len(soln)):
        if suitability_mat[i][tj]> s_min:
            viable_robots.append(i)
    
    if len(viable_robots)<=1:
        return soln
    else:
        r2 = np.random.choice(viable_robots)
        while r1==r2:
            r2 = np.random.choice(viable_robots)
        
        task_bounds = task_range(inter_task, assigned_tasks_by_idx, tj)
        
        if task_bounds[1] >=task_bounds[0]+1:
            if task_bounds[1] >= len(assigned_tasks_by_idx[r2]):
                try:
                    new_task_idx = np.random.randint(task_bounds[0]+1, len(assigned_tasks_by_idx[r2]))
                except:
                    return soln
            #print(tj)
            #print(task_bounds[0]+1, task_bounds[1])
            try:
                new_task_idx = np.random.randint(task_bounds[0]+1, task_bounds[1])
                if new_task_idx >= len(assigned_tasks_by_idx[r2]):
                    new_task_idx = len(assigned_tasks_by_idx[r2])-1
            except:
                return soln
        else:
            return soln
        
        
        assigned_tasks_by_idx[r1] = np.delete(assigned_tasks_by_idx[r1], np.where(assigned_tasks_by_idx[r1]==tj))
        
        assigned_tasks_by_idx[r2] = np.insert(assigned_tasks_by_idx[r2], new_task_idx, tj)
        
        
        #print(" ")
        for i in range(len(soln)):
            assigned_tasks_by_idx[i] = assigned_tasks_by_idx[i]+len(soln)
            assigned_tasks_by_idx[i] = np.insert(assigned_tasks_by_idx[i], 0, i)
        
        return assigned_tasks_by_idx


def crossover(chromo1, chromo2, inter_task):
    
    n_robots = len(chromo1)
    chr1 = copy.deepcopy(chromo1)
    chr2 = copy.deepcopy(chromo2)
    assigned_tasks_by_idx = to_TA(chromo1)
    for index in range(n_robots):
        robot1, robot2 = chr1[index], chr2[index]

        for i in range(1,min(len(robot1),len(robot2))):
            if robot2[i] in robot1:
                dependants = inter_task[robot2[i]-n_robots]["deps"]
                prereqs = inter_task[robot2[i]-n_robots]["prereqs"]
                
                #regular crossover if no prereqs or dependants
                if len(prereqs) == 0 and len(dependants) == 0:
                    #print(robot2[i]-n_robots)
                    robot1[i], robot1[robot1.tolist().index(robot2[i])] = robot1[robot1.tolist().index(robot2[i])], robot1[i]
                else:
                    #if there are dependants or prereqs, check that swapping doesnt break constraint
                    t1 = robot1[robot1.tolist().index(robot2[i])]
                    t2 = robot1[i]
                    t1_new_idx = i
                    t2_new_idx = robot1.tolist().index(robot2[i])
                    TR1 = task_range(inter_task, assigned_tasks_by_idx, t1-n_robots)
                    TR2 = task_range(inter_task, assigned_tasks_by_idx, t2-n_robots)
                    #if constraints still satisfied, then swap
                    if t1_new_idx>=TR1[0] and t1_new_idx<=TR1[1] and t2_new_idx>=TR2[0] and t2_new_idx<=TR2[1]:
                        #print("yes")
                        robot1[i], robot1[robot1.tolist().index(robot2[i])] = robot1[robot1.tolist().index(robot2[i])], robot1[i]
                    else:
                        #if constraint no longer satisfied then don't swap
                        #print("no")
                        pass
                
    return(chr1)

def mutate_local2(soln, inter_task):
    n_robots = len(soln)
    assigned_tasks_by_idx = [route[1:]-n_robots for route in soln]
    ri = np.random.randint(0,len(soln))

    tj = random.choice(assigned_tasks_by_idx[ri])

    task_bounds = task_range(inter_task, assigned_tasks_by_idx, tj)
    
#     print(task_bounds)
    if task_bounds[1] >=task_bounds[0]+1:
        new_task_idx = np.random.randint(task_bounds[0], task_bounds[1])
        
    else:
        return soln
    assigned_tasks_by_idx[ri] = np.delete(assigned_tasks_by_idx[ri], np.where(assigned_tasks_by_idx[ri]==tj))
    #print(np.delete(assigned_tasks_by_idx[ri], np.where(assigned_tasks_by_idx[ri]==tj)))
    assigned_tasks_by_idx[ri] = np.insert(assigned_tasks_by_idx[ri], new_task_idx,tj)
    #pprint.pprint(assigned_tasks_by_idx)
    for i in range(len(soln)):
        assigned_tasks_by_idx[i] = assigned_tasks_by_idx[i]+len(soln)
        assigned_tasks_by_idx[i] = np.insert(assigned_tasks_by_idx[i], 0, i)
    return assigned_tasks_by_idx

def genetic_algorithm_iteration(muta_prob, crossover_prob, population_in, adj, R0,
                                suitability_mat, suitability_dict, s_min, inter_task, T):
    """
    inputs: float muta_prob, float crossover_prob, list[array] population, array adj
    outputs: array new_population
    description: performs one iteration of genetic algorithm to generate new population
    generates new population by selecting best of previous population to crossover
    applies mutations to introduce diversity
    """
    # Tournament selection
    # Set the size of the tournament to be the same as the population size
    k = len(population_in)
    # Select the top 60% of the population to be parents in the tournament
    j = int(0.6*k)
    n_robots = len(population_in[0])
    n_nodes = adj.shape[0]
    population = copy.deepcopy(population_in)
    
    # Remove worst individuals from the population at random until the population size is reduced to k
    for _ in range(k - j):
        worst_chromosome_score = fitness(population[0], adj)
        worst_chromosome_index = 0
        for i in range(1,len(population)):
            if fitness(population[i], adj) > worst_chromosome_score:
                worst_chromosome_score = fitness(population[i], adj)
                worst_chromosome_index = i
        del population[-worst_chromosome_index]
 
    #add new population randomly
    for _ in range(len(population_in)- len(population)):
        random_alloc = generate_random_alloc(R0, T, suitability_dict, s_min)
        population.append(random_alloc)
        
    #apply mutation
    for _ in range(2):
        for idx in range(len(population)):
            if float(np.random.random(1)) < muta_prob:
                new_chromosome = copy.deepcopy(population[idx])
                new_chromosome = take_mutate2(new_chromosome, suitability_mat, s_min, inter_task)
                if fitness(new_chromosome,adj) < fitness(population[idx],adj):
                    population[idx] = new_chromosome  

    #apply crossover operation
    for idx1 in range(len(population)):
        if float(np.random.random(1)) < crossover_prob:
            idx2 = np.random.randint(0,len(population))
            if idx1==idx2:
                idx2 = np.random.randint(0, len(population))
            child1 = copy.deepcopy(population[idx1])
            child2 = copy.deepcopy(population[idx2])

            child1 = crossover(population[idx1], population[idx2], inter_task)
            child2 = crossover(population[idx2], population[idx1], inter_task)

            if fitness(child1,adj) < fitness(population[idx1], adj):
                population[idx1] = child1
            if fitness(child2,adj) < fitness(population[idx2], adj):
                population[idx2] = child2
  
    return(population)

def calculate_robot_cost(r, TS):
    """
    inputs: robotAgent r, List[Task] TS
    outputs: float routeCost
    description: calculates the cost for robot to do all assigned tasks
    """
    cost = 0
    if len(TS)>0:
        for i in range(len(TS)-1):

            distance = np.sqrt((TS[i+1].x-TS[i].x)**2+(TS[i+1].y-TS[i].y)**2)
            cost = cost + distance + TS[i].taskCost

        cost = cost +TS[len(TS)-1].taskCost + np.sqrt((r.x-TS[0].x)**2+(r.y-TS[0].y)**2)
    
    return cost

def update_agent_states(R0,Af):
    R_update = copy.deepcopy(R0)
    for r in R_update.keys():
        R_update[r].x = Af[r][-1].x
        R_update[r].y = Af[r][-1].y
        R_update[r].constraintUsage = calculate_robot_cost(R0[r],Af[r])
    return R_update

def sum_allocation(A1,A2):
    A3 = copy.deepcopy(A1)
    for r in A3.keys():
        for t in A2[r]:
            A3[r].append(t)
    return A3