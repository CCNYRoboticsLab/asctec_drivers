cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories (${CMAKE_CURRENT_BINARY_DIR})

# create autopilot library

rosbuild_add_library (autopilot src/autopilot.cpp
                                src/serial_interface.cpp
                                src/crc16.cpp 
                                src/telemetry.cpp)

# create autopilot_nodelet library

rosbuild_add_library (autopilot_nodelet src/autopilot_nodelet.cpp)
target_link_libraries (autopilot_nodelet autopilot)

# create autopilot_node executable

rosbuild_add_executable(autopilot_node src/autopilot_node.cpp)
target_link_libraries (autopilot_node autopilot)
