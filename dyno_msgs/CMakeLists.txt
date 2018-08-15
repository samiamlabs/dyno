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
    Delivery.msg
    DeliveryArray.msg
    Location.msg
    LocationArray.msg
    Object.msg
    ObjectArray.msg
    Robot.msg
    RobotArray.msg
    RobotAtLocation.msg
    WorldStateEvent.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    AddCurrentLocation.srv
    GetLocations.srv
    GetObjects.srv
    GetRobots.srv
    GetRobotsAtLocations.srv
    SetObjectGrabber.srv
    SetObjectMoving.srv
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
