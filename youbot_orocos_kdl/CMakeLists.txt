cmake_minimum_required(VERSION 3.0.2)
project(youbot_orocos_kdl)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(
  -O3
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  youbot_description
  kdl_conversions
  eigen_conversions
  kdl_parser
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES youbot_orocos_kdl
  CATKIN_DEPENDS
    kdl_parser
    roscpp
    rospy
    std_msgs
    kdl_conversions
  DEPENDS EIGEN3 orocos_kdl
)

## Build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(forward_kinematics
  src/forward_kinematics.cpp
)

add_dependencies(forward_kinematics ${catkin_EXPORTED_TARGETS})

target_link_libraries(forward_kinematics
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

## Install ##

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
