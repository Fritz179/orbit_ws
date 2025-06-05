cmake_minimum_required(VERSION 3.0.2)
project(bb8)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(PkgConfig REQUIRED)

find_library(WS2811_LIBRARY
             NAMES ws2811
             HINTS /usr/local/lib /lib /usr/lib
             # You might need PATH_SUFFIXES on some systems if lib is in a multiarch dir
             # PATH_SUFFIXES arm-linux-gnueabihf aarch64-linux-gnu
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
  /usr/local/include/ws2811
)

add_executable(${PROJECT_NAME}
  src/bb8_node.cpp 
  src/bb8.cpp 
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${WS2811_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
    PROGRAMS
    src/leds.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)