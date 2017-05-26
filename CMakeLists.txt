cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
