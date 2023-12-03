cmake_minimum_required(VERSION 3.0.2)
project(offboard_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
cs_add_library(trajectory_generator
  src/trajectory_generator.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(offboard_node src/offboard_node.cpp)
# target_link_libraries(offboard_node ${catkin_LIBRARIES})
cs_add_executable(offboard_node
  src/offboard_node.cpp
  src/trajectory_generator.cpp
  )

#############
## Install ##
#############
cs_install()

##########
# EXPORT #
##########
cs_export()
