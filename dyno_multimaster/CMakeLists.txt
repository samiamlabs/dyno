cmake_minimum_required(VERSION 2.8.3)
project(dyno_multimaster)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    RoconRobotPose.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()
