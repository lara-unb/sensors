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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(talker src/talker.cpp)
#rosbuild_add_executable(listener src/listener.cpp)
rosbuild_add_executable(videre_camera src/videre_camera_node.cpp src/videre_camera.cpp src/videre_log.cpp src/image_display.cpp src/camera_parameters.cpp src/stereo_data.cpp) #src/colormap.cpp src/helper.cpp)

# Enable pkg-config in cmake
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Definitions from original Makefile
include_directories(/usr/local/lib/svs/src)
link_directories(/usr/local/lib/svs/bin)
target_link_libraries(videre_camera svs svscalc svscap)