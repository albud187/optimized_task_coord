This repo contains the code for my master's thesis - "Optimized Task Coordination for Heterogenous Multi-Robot Systems", which formulated and implemented a solution for for multiple robot task allocation.

It uses a combination of a deterministic greedy algorithm and metaheuristic genetic algorithm to assign tasks to a team of UGVs in an optimal manner. 
The tasks considered are "go to goal" and "patrol" tasks. The UGVs considered are turtlebots and perform the tasks in a collision free manner using reciprocal velocity obstacles.

# File Structure

This repo contains 3 ros packages that are intended to be exectued from a docker container (Dockerfile included, along with build and run scripts):

`mrta_admin` - launch files, robots and tasks descriptions. Additional information on the mrta_admin package is on [mrta_admin/README.md](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_admin/README.md).

`mrta_main` - Task allocation algorithm. Additional information on the mrta_main package is on [mrta_main/README.md](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_main/README.md). and describes the file structure / usage of this package.

`ugv_action_model` - Motion planning and state management for UGVs. Additional inforamtion on the mrta_main package is on [ugv_action_model/README.md](https://github.com/albud187/optimized_task_coord/blob/main/src/ugv_action_model/README.md). and describes the file structure / usage of this package.


### _**Local Machine**_
```
/current_directory/optimized_task_coord
├── src/
│   ├── mrta_admin
│   ├── mrta_main
│   ├── ugv_action_model

```

When running the container using `dockerrun.sh`, the project directory is mounted as `/workdir`. This directory is treated as a ros2 workspace.
### _**Docker Container**_
```
/workdir
├── src/
│   ├── mrta_admin
│   ├── mrta_main
│   ├── ugv_action_model
```

## Setup

This repository is intended to be run using the provided docker container. In order to run it without the container, you will have to install the dependancies as well as change file path variables in the .py scripts. A list of dependancies is not provided, but can be reverse engineered from the provided `Dockerfile` and `requirements.txt`

1 - Clone the repo and change directory into the repo:

`git clone git@github.com:albud187/optimized_task_coord.git`

`cd optimized_task_coord`

2 - Build the container:

`sh dockerbuild.sh`

3 - Run the container:

`sh dockerrun.sh`

4 - While inside the container on project directory `/workdir`, build the ros pakages:

`cd /workdir`

`catkin_make`

`source devel/setup.bash`

5 - Make all `.py` files executable:

`chmod +x ./*`

## Demos

Unless otherwise stated, it is assumed that the setup is completed and you are in the docker container on `/workdir`

All instructions here concern running a demo scenario (31T-6R) included in this repo. Instructions to create your own scenario are on [mrta_admin/README.md](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_admin/README.md)

### _**rqt Demo**_

1 - Open two terminals and run the docker container. On both terminals execute:

`sh dockerrun.sh`

2 - On terminal 1 execute:

`roslaunch ugv_action_model multi_ugv.launch`

This will launch a gazebo simulation with 6 turtlebots.

3 - On terminal 2 execute:

`rqt`

We will use rqt to publish to the `r_n/task` topics, where `r_n` is the namespace of an individual robot and ranges from `r_0` to `r_5` for this demo.

4 - The topics of `r_0/task` to `r_5/task` are of type PoseStamped, and describe tasks as follows:
`PoseStamped.header.frame_id`: type of task. String. Set it to "A" for a goal to goal task. Set it to "D" for a patrol task.

`PoseStamped.pose.position.x`: x coordinate of task location. Float.

`PoseStamped.pose.position.y`: y coordinate of task location. Float.

`PoseStamped.pose.orientation.x`: Length of patrol route. Only applicatible if `PoseStamped.header.frame_id = "D"`

`PoseStamped.pose.orientation.w`: Task completion indicator. Set it to 1 (or anything not 0) otherwise the robot will not move.

>>need image of rqt and robots moving

5 - Publish the task topic. The robot corresponding to the task topic will execute the task.


### _**Multiple Robot Task Allocation Demo**_

1 - Open two terminals and run the docker container. On both terminals execute:

`sh dockerrun.sh`

2 - On terminal 1 execute:

`roslaunch ugv_action_model multi_ugv.launch`

This will launch a gazebo simulation with 6 turtlebots.

3 - On terminal 2 execute:

`roslaunch mrta_admin main.launch scenario:=31T-6R`

![Gazebo Simulation](https://github.com/albud187/optimized_task_coord/blob/main/.repo_images/gazebo_scenario.PNG)

![Example Task Allocation](https://github.com/albud187/optimized_task_coord/blob/main/.repo_images/example_task_allocation.PNG)

This will execute the optimization algorithms to assign tasks described in the `src/scenarios/31T-6R` to the 6 turtlebots. This scenario has 31 tasks and 6 robots. The tasks are described in `src/scenarios/31T-6R/tasks.csv`, the suitability between the robots and tasks are described in `src/scenarios/31T-6R/suitabilities.csv`. The type of robots and their work capacity are described in `src/scenarios/31T-6R/agents.csv`. Instructions on creating your own scenario are described in [mrta_admin/README.md](https://github.com/albud187/optimized_task_coord/blob/main/src/mrta_admin/README.md).

