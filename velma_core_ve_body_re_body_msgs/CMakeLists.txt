cmake_minimum_required(VERSION 2.8.6)

project(velma_core_ve_body_re_body_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs message_generation actionlib_msgs)

add_message_files(FILES
  CommandArm.msg
  CommandSimple.msg
  CommandMotor.msg
  Command.msg
  StatusArm.msg
  StatusFT.msg
  StatusMotor.msg
  StatusArmFriIntf.msg
  StatusArmFriRobot.msg
  Status.msg
  )
  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)
