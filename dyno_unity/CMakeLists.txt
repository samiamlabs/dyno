cmake_minimum_required(VERSION 2.8.3)
project(dyno_unity)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    controller_manager
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(forklift src/forklift.cpp)
target_link_libraries(forklift ${catkin_LIBRARIES})

catkin_package()
