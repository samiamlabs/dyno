cmake_minimum_required(VERSION 2.8.3)

project(dyno_world_state)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    Location.msg
    LocationArray.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    AddCurrentLocation.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()
