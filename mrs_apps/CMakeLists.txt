cmake_minimum_required(VERSION 2.8.3)
project(mrs_apps)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(DEPENDS phantom_omni)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup() 
