cmake_minimum_required(VERSION 2.8.3)
project(labust_mission)

find_package(catkin REQUIRED COMPONENTS
 # decision_making
 # decision_making_parser
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  snippets
  tf2_ros
  auv_msgs
  misc_msgs
  cmake_modules
  labust_primitives
  labust_diagnostics
  std_srvs
  actionlib
  actionlib_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(GeographicLib REQUIRED)

find_path(TinyXML2_INCLUDE_DIR tinyxml2.h
	PATH_SUFFIXES include
	PATHS /usr/local /usr)

find_library(TinyXML2_LIBRARY
	tinyxml2
	PATH_SUFFIXES lib64 lib
	PATHS /usr/local /usr)
	
## Messages	
	add_action_files(
  DIRECTORY action
  FILES 
  RobotTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS decision_making decision_making_parser roscpp labust_primitives
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp labust_primitives
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
  ${Boost_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS})
  

add_library(decision_making_ros src/decision_making/ROSTask.cpp)

## Main mission control nodes
if (TinyXML2_LIBRARY)
	include_directories(${TinyXML2_INCLUDE_DIR})

  add_executable(mission_execution src/mission_execution.cpp)
  add_dependencies(mission_execution misc_msgs_gencpp auv_msgs_gencpp labust_mission_generate_messages_cpp)
	target_link_libraries(mission_execution ${catkin_LIBRARIES} ${TinyXML2_LIBRARY} decision_making_ros)

  add_executable(mission_parser src/mission_parser.cpp)
  add_dependencies(mission_parser misc_msgs_gencpp auv_msgs_gencpp)
	target_link_libraries(mission_parser ${catkin_LIBRARIES} ${TinyXML2_LIBRARY})

  add_executable(data_event_manager src/data_event_manager.cpp)
  add_dependencies(data_event_manager misc_msgs_gencpp auv_msgs_gencpp)
	target_link_libraries(data_event_manager ${catkin_LIBRARIES} ${TinyXML2_LIBRARY})

  add_executable(commander src/commander.cpp)
  add_dependencies(commander misc_msgs_gencpp auv_msgs_gencpp)
	target_link_libraries(commander ${catkin_LIBRARIES} ${TinyXML2_LIBRARY})
else()
  message("WARNING" "Custom installation of TinyXML2 library is missing.")
endif()

## Caddy mission control node
add_executable(caddy_missions src/caddy_missions.cpp)
add_dependencies(caddy_missions misc_msgs_gencpp auv_msgs_gencpp)
target_link_libraries(caddy_missions ${catkin_LIBRARIES})

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
# install(TARGETS caddy_state_machine caddy_state_machine_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_caddy_state_machine.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
