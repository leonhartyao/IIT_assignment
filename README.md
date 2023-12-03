# Indoor UAV assignment Docker

This is the indoor uav docker that should be used for assignments related to integration. 

## Contents

    Ubuntu 20
    ROS Noetic
    PX4 Software in the loop (with depth camera)
    Gazebo Classic
    AWS small house world

## How to use

    1. ./build
    2. ./attach to run and attach
    3. ./scripts/run to run the simulation (inside docker)

There is no bashrc file inside the docker, so the run script does all the sourcing necessary to run all the scripts.

The source files should be inside assignment/src and properly built/sourced.

Feel free to add dependencies on the docker build if they are necessary for the task.

Don't forget: We should be able to run the assignment easily, so please document all changes and all steps necessary to run the deliverables.

## Assignment results

- Trajectory planner is implemented successfully.
- Flight mission is implemented successfully. OFFBOARD -> Arm -> Trajectory following -> HOLD -> LAND -> Disarm
- Simple PID control implemented, not integrated, not tuned. Input: desired position, current position, dt. Output: desired attitude, desired thrust.
- No time for advanced model-based controller.

## How to run the assignment

Hints:

- assignment folder is mount as /workspace/catkin_ws in contrainer
- start new terminal attached to container with `docker exec -it iit-assignment /bin/bash`
- Don't forget to source `/opt/ros/noetic/setup.bash` and `install/setup.bash`.
- .catkin_tools is added to the repo for convenience
- Building nlopt will take a while, keep patient and sorry for inconvenience.
- For unclear reason: if UAV doesn't takeoff after armed for a while, keep SIL and rerun the offboard_control. It always succeeds in a second run.

1. git clone
2. `./build.sh`
3. `./attach.sh`
4. `catkin build offboard_control`
5. `./scripts/run.sh`
6. `rosrun offboard_control offboard_node` (if UAV is armed but doesn't take off, ctrl+c and rerun)
7. Check screen out and gazebo simulation.
8. Screen logs was saved in`./results`
