cmake_minimum_required(VERSION 2.8.3)
project(sun_ros_utils_nodes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sun_systems_lib
  sun_ros_msgs
  geometry_msgs
  tf2_ros
  nodelet
  pluginlib
  toon_ros_msg_conversion
  sun_math_toolbox
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES
#  std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  roscpp
  INCLUDE_DIRS include
  LIBRARIES transform_wrench_nodelet wrench_difference_nodelet transform_wrench_virtual_stick_nodelet
 CATKIN_DEPENDS  nodelet pluginlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
${catkin_INCLUDE_DIRS}
)

add_library(transform_wrench_nodelet
  src/${PROJECT_NAME}/transform_wrench_nodelet.cpp
)
add_dependencies(transform_wrench_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_wrench_nodelet
${catkin_LIBRARIES}
)

add_library(wrench_difference_nodelet
  src/${PROJECT_NAME}/wrench_difference_nodelet.cpp
)
add_dependencies(wrench_difference_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_difference_nodelet
${catkin_LIBRARIES}
)

add_library(transform_wrench_virtual_stick_nodelet
  src/${PROJECT_NAME}/transform_wrench_virtual_stick_nodelet.cpp
)
add_dependencies(transform_wrench_virtual_stick_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_wrench_virtual_stick_nodelet
${catkin_LIBRARIES}
)


# Declare a C++ library
#  add_library(${PROJECT_NAME}
#    src/${PROJECT_NAME}/FrequencyScaler.cpp
#  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/sun_ros_utils_nodes_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sun_ros_utils_nodes.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#########################################
add_executable(filter_wrench_stamped
  src/filter_wrench_stamped_node.cpp
)
add_dependencies(filter_wrench_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_wrench_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(filter_float32
  src/filter_float32_node.cpp
)
add_dependencies(filter_float32 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_float32
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(filter_float64_stamped
  src/filter_float64_stamped_node.cpp
)
add_dependencies(filter_float64_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_float64_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(scale_joint_state
  src/scale_joint_state_node.cpp
)
add_dependencies(scale_joint_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scale_joint_state
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(scale_wrench_stamped
  src/scale_wrench_stamped_node.cpp
)
add_dependencies(scale_wrench_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scale_wrench_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(mov_av_wrench_stamped
  src/mov_av_wrench_stamped_node.cpp
)
add_dependencies(mov_av_wrench_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mov_av_wrench_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(wrench_bias
  src/wrench_bias_node.cpp
)
add_dependencies(wrench_bias ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_bias
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(wrench_stamped_frequency_scaler
  src/frequency_scaler/wrench_stamped_frequency_scaler_node.cpp
)
add_dependencies(wrench_stamped_frequency_scaler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_stamped_frequency_scaler
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(twist_stamped_frequency_scaler
  src/frequency_scaler/twist_stamped_frequency_scaler_node.cpp
)
add_dependencies(twist_stamped_frequency_scaler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_stamped_frequency_scaler
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(transform_wrench
  src/transform_wrench_node.cpp
)
add_dependencies(transform_wrench ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_wrench
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(transform_pose
  src/transform_pose_node.cpp
)
add_dependencies(transform_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_pose
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(filter_twist_stamped
  src/filter_twist_stamped_node.cpp
)
add_dependencies(filter_twist_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_twist_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(differentiate_float64_stamped
  src/differentiate_float64_stamped_node.cpp
)
add_dependencies(differentiate_float64_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(differentiate_float64_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(integrate_float64_stamped
  src/integrate_float64_stamped_node.cpp
)
add_dependencies(integrate_float64_stamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(integrate_float64_stamped
  ${catkin_LIBRARIES}
)
#########################################
#########################################
add_executable(wrench_difference
  src/wrench_difference_node.cpp
)
add_dependencies(wrench_difference ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_difference
  ${catkin_LIBRARIES}
)
#########################################