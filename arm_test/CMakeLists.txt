cmake_minimum_required(VERSION 2.8.3)
project(arm_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  gazebo_world_plugin_loader
  std_msgs
  moveit_msgs
  gazebo_ros
  object_msgs
  object_msgs_tools
  eigen_conversions
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###############################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
 add_message_files(
   FILES
   graspObject.msg
   graspposition.msg

 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   go.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   roscpp
   geometry_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(

  CATKIN_DEPENDS cmake_modules interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(grasp_decider_1 src/grasp_decider_1.cpp)
target_link_libraries(grasp_decider_1
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS grasp_decider_1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasp_decider_2 src/grasp_decider_2.cpp)
target_link_libraries(grasp_decider_2
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS grasp_decider_2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasp_decider_3 src/grasp_decider_3.cpp)
target_link_libraries(grasp_decider_3
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS grasp_decider_3 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasp_decider_4 src/grasp_decider_4.cpp)
target_link_libraries(grasp_decider_4
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS grasp_decider_4 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasp_decider_5 src/grasp_decider_5.cpp)
target_link_libraries(grasp_decider_5
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS grasp_decider_5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(ppclient src/ppclient.cpp)
target_link_libraries(ppclient
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS ppclient DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene1 src/planning_scene1.cpp)
target_link_libraries(planning_scene1
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS planning_scene1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene2 src/planning_scene2.cpp)
target_link_libraries(planning_scene2
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS planning_scene2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



add_executable(pickplace1 src/pickplace1.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(pickplace1
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS pickplace1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pickplace2 src/pickplace2.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(pickplace2
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
install(TARGETS pickplace2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


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
# install(TARGETS seven_dof_arm_test seven_dof_arm_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_seven_dof_arm_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

