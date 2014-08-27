cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_find_ros_package(cds_control_nodes)
include_directories(${cds_control_nodes_PACKAGE_PATH}/include)
rosbuild_add_library(${PROJECT_NAME} 
  ${cds_control_nodes_PACKAGE_PATH}/src/arm_kinematics.cpp 
  ${cds_control_nodes_PACKAGE_PATH}/src/kdl_epfl_conversions.cpp 
  ${cds_control_nodes_PACKAGE_PATH}/src/cds_cartesian_wrapper.cpp
)

rosbuild_add_executable(cds_controller_pr2 src/cds_controller_pr2.cpp)
target_link_libraries(cds_controller_pr2 ${PROJECT_NAME})

rosbuild_add_boost_directories()
rosbuild_find_ros_package(ias_knowledge_base)
rosbuild_include(rosprolog utils)

set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

# replace path to local OWL files                                                                                                                                       
rosbuild_find_ros_package(lesson01)
set( LOCAL_PACKAGE_PATH ${lesson01_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

rosbuild_add_executable(eeframe_tf_broadcaster src/eeframe_tf_broadcaster.cpp)
rosbuild_add_executable(cartesian_controller src/cartesian_controller.cpp)
