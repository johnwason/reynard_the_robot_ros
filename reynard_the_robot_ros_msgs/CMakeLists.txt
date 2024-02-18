cmake_minimum_required(VERSION 3.0.2)
project(reynard_the_robot_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  ReynardState.msg
)

add_service_files(
  FILES
  Drive.srv
  GetColor.srv
  GetPosition.srv
  SetPosition.srv
  SetColor.srv
  Teleport.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
)



