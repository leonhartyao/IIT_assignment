FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y python3-catkin-tools python3-osrf-pycommon \
     python-is-python2 python3-pip git

RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    libgstreamer-plugins-base1.0-dev \
    vim

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh &&\
    sudo bash ./install_geographiclib_datasets.sh  

RUN pip3 install kconfiglib
RUN pip3 install jinja2 
RUN pip3 install jsonschema
RUN pip3 install packaging
RUN pip3 install toml
RUN pip3 install pyros-genmsg
RUN pip3 install future

COPY assignment/ /workspace/assignment/.
RUN . /opt/ros/noetic/setup.sh
RUN cd /workspace/assignment/resources/ && git clone -b v1.13.3 https://github.com/PX4/PX4-Autopilot 
RUN cd /workspace/assignment/resources/PX4-Autopilot && git apply /workspace/assignment/scripts/0001-Added-depth-iris.patch
RUN cd /workspace/assignment/resources/PX4-Autopilot && DONT_RUN=1 make px4_sitl gazebo
RUN cd /workspace/assignment/resources && git clone -b ros1 https://github.com/aws-robotics/aws-robomaker-small-house-world
WORKDIR /workspace/assignment/
