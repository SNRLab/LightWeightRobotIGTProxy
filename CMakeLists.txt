PROJECT(LightWeightRobotIGTProxy)

cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(OpenIGTLink REQUIRED)

include(${OpenIGTLink_USE_FILE})

ADD_EXECUTABLE(LightWeightRobotIGTProxy  LightWeightRobotIGTProxy.cxx)
TARGET_LINK_LIBRARIES(LightWeightRobotIGTProxy OpenIGTLink)

ADD_EXECUTABLE(VirtualSlicerRobotCommand  VirtualSlicerRobotCommand.cxx)
TARGET_LINK_LIBRARIES(VirtualSlicerRobotCommand OpenIGTLink)



