cmake_minimum_required(VERSION 2.8.3)
project(asctec_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ControllerOutput.msg
  CtrlInput.msg
  GPSData.msg
  GPSDataAdvanced.msg
  IMUCalcData.msg
  IMURawData.msg
  LLStatus.msg
  RCData.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs message_runtime
  )

include_directories(include
   ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
