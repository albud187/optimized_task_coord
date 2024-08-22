This ROS package is an implementation of an action model for collaborative task execution with a team of UGVs for  chapter 4 and annex A3 of my [master's thesis](https://github.com/albud187/optimized_task_coord/blob/main/.thesis/Budiman_Alfa_2023_Thesis.pdf)


# Key File Overview

The key files are as follows:

```
/workdir
├── src/
│   ├── ugv_action_moodel
│   │   ├── launch
│   │   │   ├── ugv.launch
│   │   │   ├── ugv_2.launch
│   │   │   ├── ugv_6.launch
│   │   ├── scripts
│   │   │   ├── RVO_planner.py
│   │   │   ├── task_list_sub.py
│   │   │   ├── status_task_manager.py
```
### _**ROS Node Overview**_
The rest of the files in this ros package primarily concern with ROS communications required to faciliate the action model. It is also important to note that certain aspects such as namespaces and details relavant to task exection are described in [mrta_main/readme](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md).

- `RVO_planner.py` : ROS node that implements reciprocal velocity obstacles.

-`task_list_sub.py`: ROS node that subscribes to list of tasks published by `mrta_main/scripts/task_list_pub.py`. After recieving a task list, the `task_list_sub.py` node checks the robot's availability and if avaialble, publishes the next task to `status_task_manager.py`

-`status_task_manager.py`: ROS node that describes behaviour to be executed for specific task types. Therefore, it is critical that the task types described in this file match the scenario descriptions in `mrta_main/scenarios` as described in [mrta_main/readme](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md).

Currently, `status_task_manager.py` is set to use task types of "D", "E", "F", "G", "A". If the task type is "D", the robot will perform a patrol task, which entails driving in a square pattern centered around the task location as described in [mrta_main/readme](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md). If the task type is  "D", "E", "F" or "G", it is simply a go to goal task, which entails moving to the task location.

### _**Launch File Overview*_

-`ugv.launch`: Launches the ROS nodes needed to manage the status of one UGV
-`ugv_2.launch`: Launches a gazebo simulation of 2 turtlebots, as well as instances of `ugv.launch` for each turtlebot with their own unique namespace. Launches `RVO_planner.py` for reciprocal velocity obstacles. Contains launch arguments for the start position of the turtlebots.
-`ugv_6.launch`: same as `ugv_2.launch` but with 6 turtlebots.


