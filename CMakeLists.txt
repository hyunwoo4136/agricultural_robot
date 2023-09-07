cmake_minimum_required(VERSION 2.8.3)
project(agricultural_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(agricultural_robot src/agricultural_robot.cpp)

target_link_libraries(agricultural_robot
  ${catkin_LIBRARIES}
)

