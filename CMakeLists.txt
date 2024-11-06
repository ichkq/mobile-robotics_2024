cmake_minimum_required(VERSION 2.8.3)
project(mobile-robotics_2024)
add_definitions(-std=c++17 -Wall -pedantic -Wextra)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  cv_bridge
  std_msgs
  message_generation
  geometry_msgs
  image_transport
)
find_package(OpenCV REQUIRED)



################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
#    pos.msg
# )
## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    std_msgs
# )
## Generate the dynamic_reconfigure
# generate_dynamic_reconfigure_options(
#    cfg/turtle.cfg
#    #...
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES mobile-robotics_2024
  CATKIN_DEPENDS roscpp rospy message_runtime dynamic_reconfigure
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

#################
## Executables ##
#################
include_directories(${OpenCV_INCLUDE_DIRS})


