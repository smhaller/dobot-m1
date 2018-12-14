cmake_minimum_required(VERSION 2.8.3)
project(dobot_m1_demos)

find_package(catkin REQUIRED COMPONENTS
  rospy
  trajectory_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

