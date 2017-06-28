cmake_minimum_required(VERSION 2.8.3)
project(joystick_control)
add_definitions(-std=c++11)
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


add_executable(youbot_control
  src/youbot_control.cpp
  /home/macs/catkin_ws/src/youbot_Kinematics/src/youbot_kinematics.cpp
  /home/macs/catkin_ws/src/youbot_Kinematics/src/geometry.cpp
)
target_link_libraries(youbot_control
  ${catkin_LIBRARIES}
)