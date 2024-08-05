This repo contains the code for my master's thesis - "Optimized Task Coordination for Heterogenous Multi-Robot Systems", which formulated and implemented a solution for for multiple robot task allocation.

It uses a combination of a deterministic greedy algorithm and metaheuristic genetic algorithm to assign tasks to a team of UGVs in an optimal manner. 
The tasks considered are "go to goal" and "patrol" tasks.
The UGVs considered are turtlebots.

# File Structure

This repo contains 3 ros packages that are intended to be exectued from a docker container (Dockerfile included, along with build and run scripts):

`mrta_admin` - launch files, robots and tasks descriptions

`mrta_main` - Task allocation algorithm

`ugv_action_model` - Motion planning and state management for UGVs


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

