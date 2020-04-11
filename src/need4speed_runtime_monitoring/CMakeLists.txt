cmake_minimum_required(VERSION 2.8.3)
project(need4speed_runtime_monitoring)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vesc_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES need4speed_runtime_monitoring
#  CATKIN_DEPENDS roscpp rospy std_msgs vesc_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(need4speed_average_vesc src/need4speed_average_vesc.cpp)
target_link_libraries(need4speed_average_vesc ${catkin_LIBRARIES})

