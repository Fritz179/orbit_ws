cmake_minimum_required(VERSION 3.0.2)
project(tb6612fng)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PIGPIO REQUIRED pigpio)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tb6612fng
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PIGPIO_INCLUDE_DIRS}
)

add_library(tb6612fng
  src/tb6612fng.cpp
)

target_link_libraries(tb6612fng
  ${catkin_LIBRARIES}
  ${PIGPIO_LIBRARIES}
)