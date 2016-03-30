cmake_minimum_required(VERSION 2.8.3)
project(random_gps_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

## Declare ROS messages
#add_message_files(FILES
#  NavSatStatus
#  NavSatFix
#)

## Generate added messages
#generate_messages(DEPENDENCIES
#  sensor_msgs
#)

## Decare a catkin package
catkin_package()

## Build gps_publisher
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gps_publisher src/gps_publisher.cpp)
target_link_libraries(gps_publisher ${catkin_LIBRARIES})
