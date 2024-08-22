This ROS package is an implementation of an action model for collaborative task execution with a team of UGVs for  chapter 4 and annex A3 of my [master's thesis](https://github.com/albud187/optimized_task_coord/blob/main/.thesis/Budiman_Alfa_2023_Thesis.pdf)


# Key File Overview

The key files are as follows:

```
/workdir
├── src/
│   ├── ugv_action_moodel
│   │   ├── launch
│   │   ├── scripts
│   │   │   ├── RVO_planner.py
│   │   │   ├── task_list_sub.py
│   │   │   ├── status_task_manager.py
```
The rest of the files in this ros package primarily concern with ROS communications required to faciliate the action model. It is also important to note that certain aspects such as namespaces and details relavant to task exection are described in [mrta_main/readme](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md).

The `RVO_planner.py` file is a ROS node that implements reciprocal velocity obstacles.

The `task_list_sub.py` file is a ROS node that subscribes to list of tasks published by `mrta_main/scripts/task_list_pub.py`. After recieving a task list, the `task_list_sub.py` node checks the robot's availability and if avaialble, publishes the next task to `status_task_manager.py`

The `status_task_manager.py` file is a ROS node that describes behaviour to be executed for specific task types. Therefore, it is critical that the task types described in this file match the scenario descriptions in `mrta_main/scenarios` as described in [mrta_main/readme](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md).

Currently, `status_task_manager.py` is set to use task types of "D", "E", "F", "G", "A". If the task type is "D", the robot will perform a patrol task, which entails driving in a square pattern centered around the task location as described in [mrta_main/readme](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md). If the task type is  "D", "E", "F" or "G", it is simply a go to goal task, which entails moving to the task location.

The `status_task_manager.py` ultimately publishes waypoints based on the task assigned to the robot. Then, the robot's command velocity is calculated by `RVO_planner.py`, `velocity_publisher.py` and `pose_reporter.py`.

