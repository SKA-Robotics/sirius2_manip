cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius2_manip)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    std_msgs
)

install(PROGRAMS
  scripts/mock_manip.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)