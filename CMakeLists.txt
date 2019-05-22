cmake_minimum_required(VERSION 2.8.3)
project(mav_state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#catkin_simple()
#catkin_python_setup()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  ail_mav
)



## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  tracking.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs   std_msgs   geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES state_machine
 CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(traj_plan_track_action_server src/traj_plan_track_action_server.cpp)
target_link_libraries(traj_plan_track_action_server ${catkin_LIBRARIES})

