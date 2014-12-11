cmake_minimum_required(VERSION 2.8.3)
project(fetch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  rosutil
  hardware_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wallfollower
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib

  #DEPENDS orocos_kdl
  #CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs rosutil hardware_msgs
  #INCLUDE_DIRS
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fetch src/fetch.cpp)
target_link_libraries(fetch ${catkin_LIBRARIES} rosutil)
add_dependencies(fetch ${catkin_EXPORTED_TARGETS})
