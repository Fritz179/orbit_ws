cmake_minimum_required(VERSION 3.0.2)
project(bb8_zero)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(PkgConfig REQUIRED)
# pkg_check_modules(PIGPIO REQUIRED pigpio)

find_library(PIGPIOD_LIB
             NAMES pigpiod_if2
             PATHS /usr/lib /usr/local/lib
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PIGPIO_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/bb8_zero_node.cpp 
  src/bb8_zero.cpp 
  src/l298n.cpp 
  src/lx16a.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PIGPIOD_LIB}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)