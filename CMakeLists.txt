cmake_minimum_required(VERSION 3.0.2)
project(topic_state)

find_package(catkin REQUIRED COMPONENTS
  ddynamic_reconfigure
  dynamic_reconfigure
  message_generation
  nodelet
  rosbag
  roscpp
  roslib
  roslint
  rospy
  sensor_msgs
  std_msgs
)

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces"
)
roslint_cpp()
# TODO(lucasw) only this shows the error
# https://answers.ros.org/question/335608/roslint-python-errors-to-stdout/
set(ROSLINT_PYTHON_OPTS "1>&2")
# Don't see any results from these
# set(ROSLINT_PYTHON_OPTS "--show-source" "--show-pep8")
# set(ROSLINT_PYTHON_OPTS "--show-source")
# set(ROSLINT_PYTHON_OPTS "--show-pep8")
# set(ROSLINT_PYTHON_OPTS "--verbose")
# set(ROSLINT_PYTHON_OPTS "--count")
roslint_python()

# catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
  ddynamic_reconfigure
  dynamic_reconfigure
  nodelet
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(float_stats src/float_stats.cpp)
target_link_libraries(float_stats
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/average.py
  scripts/bag_msg_age.py
  scripts/diff.py
  scripts/header_stamp_to_date.py
  scripts/msg_age.py
  scripts/multi_echo.py
  scripts/ros_tui.py
  scripts/rosout_to_stdout.py
  scripts/topic_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
