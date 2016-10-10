cmake_minimum_required(VERSION 2.8.3)
project(ros_controllers)
find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(my_minimal_joint_controller src/minimal_joint_controller.cpp)
cs_add_executable(my_sin_commander src/sine_commander.cpp)
cs_install()
cs_export()
