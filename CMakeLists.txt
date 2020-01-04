cmake_minimum_required(VERSION 2.8.3)
project(topic_state)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  ddynamic_reconfigure
  message_generation
  nodelet
  roscpp
  roslint
  rospy
  std_msgs
)

# catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS ddynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(float_stats src/float_stats.cpp)
target_link_libraries(float_stats
  ${catkin_LIBRARIES}
)

# TODO(lucasw) install
