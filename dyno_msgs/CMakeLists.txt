cmake_minimum_required(VERSION 2.8.3)

project(dyno_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    Location.msg
    LocationArray.msg
    Object.msg
    ObjectArray.msg
    Delivery.msg
    DeliveryArray.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    AddCurrentLocation.srv
    GetObjects.srv
    SetObjectGrabber.srv
)

add_action_files(
  DIRECTORY
    action
  FILES
    QuadrotorPickUpParcel.action
    QuadrotorDropOffParcel.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package()
