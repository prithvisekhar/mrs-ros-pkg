cmake_minimum_required(VERSION 2.8.3)
project(mrs_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rosconsole
  rospy 
  message_generation 
  dynamic_reconfigure
  control_msgs
  controller_manager
  )



  find_package(Boost REQUIRED COMPONENTS system thread)

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    )

# Import the package name
catkin_python_setup()
#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/Parameter.cfg)

#add_dependencies(server ${PROJECT_NAME}_server) # wait for dynamic reconfigure
#add_dependencies(Tutorials ${PROJECT_NAME}_gencfg) # wait for dynamic reconfigure


#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrs_control
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
DEPENDS tinyxml
    CATKIN_DEPENDS 
      rosconsole 
      roscpp 
      dynamic_reconfigure
    LIBRARIES ${PROJECT_NAME}
)






