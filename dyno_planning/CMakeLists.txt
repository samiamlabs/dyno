cmake_minimum_required(VERSION 2.8.3)

project(dyno_planning)

find_package(catkin REQUIRED COMPONENTS
  rosplan_planning_system
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rosplan_planning_system
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(quadrotor_interface src/quadrotor_interface.cpp)
target_link_libraries(quadrotor_interface ${catkin_LIBRARIES})
add_dependencies(quadrotor_interface ${catkin_EXPORTED_TARGETS})
