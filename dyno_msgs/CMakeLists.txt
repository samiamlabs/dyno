cmake_minimum_required(VERSION 2.8.3)

project(dyno_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
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
    ObjectAtLocation.msg
    ObjectOnRobot.msg
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
    AddObjectOnRobot.srv
    GetLocations.srv
    GetMap.srv
    GetObjects.srv
    GetObjectsAtLocations.srv
    GetObjectsOnRobots.srv
    GetRobots.srv
    GetRobotsAtLocations.srv
    RemoveObjectOnRobot.srv
    SetLocations.srv
    SetObjectGrabber.srv
    SetObjectMoving.srv
    SetObjects.srv
    SetRobots.srv
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
    nav_msgs
)

catkin_package()
