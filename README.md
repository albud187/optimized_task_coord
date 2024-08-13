This repo contains the code for my master's thesis - "Optimized Task Coordination for Heterogenous Multi-Robot Systems", which formulated and implemented a solution for for multiple robot task allocation.

It uses a combination of a deterministic greedy algorithm and metaheuristic genetic algorithm to assign tasks to a team of UGVs in an optimal manner. 
The tasks considered are "go to goal" and "patrol" tasks. The UGVs considered are turtlebots and perform the tasks in a collision free manner using reciprocal velocity obstacles.

# File Structure

This repo contains 3 ros packages that are intended to be exectued from a docker container (Dockerfile included, along with build and run scripts):

`mrta_admin` - launch files, robots and tasks descriptions

`mrta_main` - Task allocation algorithm. Additional inforamtion on the mrta_main package is on <link> and describes the file structure / usage of this package.

`ugv_action_model` - Motion planning and state management for UGVs. Additional inforamtion on the mrta_main package is on <link> and describes the file structure / usage of this package.


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

## Demo

Unless otherwise stated, it is assumed that the setup is completed and you are in the docker container on `/workdir`

All instructions here concern running a demo. Instructions to create your own scenario are on <link>

### _**Multiple UGV Motion Planning Only**_

1 - Open two terminals and run the docker container. On both terminals execute:

`sh dockerrun.sh`

2 - On terminal 1 execute:

`roslaunch ugv_action_model multi_ugv.launch`

This will launch a gazebo simulation with 6 turtlebots.

3 - On terminal 2 execute:

`rqt`

We will use rqt to publish to the `r_n/task` topics, where `r_n` is the namespace of an individual robot and ranges from `r_0` to `r_5` for this demo.

## Usage (


