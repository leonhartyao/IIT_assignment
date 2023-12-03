cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
cs_add_executable(trajectory_generator
        src/trajectory_generator.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
