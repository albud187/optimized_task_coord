This ros package is an implementation of the greedy and genetic task allocation algorithm described in chapter 3 of my [master's thesis](https://github.com/albud187/optimized_task_coord/blob/main/.thesis/Budiman_Alfa_2023_Thesis.pdf)


# File Structure
```
/workdir
├── src/
│   ├── mrta_main
│   │   ├── scenarios
│   │   ├── launch
│   │   ├── scripts
│   │   │   ├── MRTA/
│   │   │   ├── task_list_pub.py
│   │   │   ├── system_monitor.py
```

The directory `src/mrta_main/scripts/MRTA` contains the python package that implement task allocation algorithm itself. These are imported by `task_list_pub.py`, which is the ROS node that generates the lists of tasks and publishes them.

The directory `src/mrta_main/scenarios` contains scenario descriptions. A scenario description consists of a subdirectory (whose name is a launch argument) that contains an `agents.csv`, `suitabilities.csv`, and `tasks.csv`.

The  `agents.csv` file lists each robot by namespace (`ns` column). The entries in the `ns` must match the namespaces assigned to the robots on the gazebo simulation. In the provided example in `scenarios/31T-6R`, the namespaces match the ones assigned in `ugv_action_model/launch/ugv_6.launch`. The entries in `agents.csv` describe the type of robot (`robotType` column), the work capacity (`actionCap` column) for each robot.

The `suitabilities.csv` file lists the suitabilitity scores by task type and robot type. Agent-task matching will only be made if the suitability score exceeds the minimum suitability requirement, which is `S_MIN` in `MRTA/_MRTA_constants.py`.

The `tasks.csv` file tasks by location (`x` , `y`), type (`taskType`), actuation cost (`taskCost`) and prerequisites (`prereqs`). This file is read and processed into the list of tasks for the task allocation algorithm.

The task types, robot types in the following files must all match and be consistent:

- `src/mrta_main/scenarios/ScenarioName/agents.csv`
  
- `src/mrta_main/scenarios/ScenarioName/suitabilities.csv`
  
- `src/mrta_main/scenarios/ScenarioName/tasks.csv`
  
- `src/ugv_action_model/scripts/status_task_manager.py`

In the provided example, `ScenarioName` is `31T-6R`.
 
