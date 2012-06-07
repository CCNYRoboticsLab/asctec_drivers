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

rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create autopilot library

rosbuild_add_library (asctec_proc src/asctec_proc.cpp)

# create autopilot_nodelet library

rosbuild_add_library (asctec_proc_nodelet src/asctec_proc_nodelet.cpp)
target_link_libraries (asctec_proc_nodelet asctec_proc)

# create autopilot_node executable

rosbuild_add_executable(asctec_proc_node src/asctec_proc_node.cpp)
target_link_libraries (asctec_proc_node asctec_proc)
