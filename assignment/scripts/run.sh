cd /workspace/assignment/resources/PX4-Autopilot 
source /opt/ros/noetic/setup.bash 
DONT_RUN=1 make px4_sitl_default gazebo_iris_depth_camera
#source /workspace/assignment/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
cd /workspace/assignment/resources/aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models
cd /workspace/assignment/scripts
roslaunch launch.launch