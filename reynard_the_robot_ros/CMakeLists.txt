cmake_minimum_required(VERSION 3.0.2)
project(reynard_the_robot_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  reynard_the_robot_ros_msgs
  std_msgs
  rospy
)

catkin_package(
CATKIN_DEPENDS geometry_msgs reynard_the_robot_ros_msgs std_msgs rospy
)

catkin_install_python(PROGRAMS
  scripts/reynard_the_robot_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
