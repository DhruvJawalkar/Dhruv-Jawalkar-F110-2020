cmake_minimum_required(VERSION 2.8.3)
project(jawalkar_runtime_monitoring)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jawalkar_runtime_monitoring
#  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jawalkar_average src/jawalkar_average.cpp)
target_link_libraries(jawalkar_average ${catkin_LIBRARIES})


