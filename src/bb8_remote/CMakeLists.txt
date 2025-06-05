cmake_minimum_required(VERSION 3.0.2)
project(bb8_remote)

find_package(catkin REQUIRED COMPONENTS
rospy
cartographer_ros_msgs
geometry_msgs
)

catkin_package()

include_directories()

catkin_install_python(
    PROGRAMS
    src/bb8_remote.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)