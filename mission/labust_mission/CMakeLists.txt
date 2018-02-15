cmake_minimum_required(VERSION 2.8.3)
project(labust_mission)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  snippets
  tf2_ros
  auv_msgs
  cmake_modules
  labust_primitive
  labust_diagnostics
  diagnostic_updater
  std_srvs
  actionlib
  actionlib_msgs
  message_generation
  geographic_msgs
  labust_msgs
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

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

## Catkin specific configuration 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp labust_primitive
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS})
  

add_library(decision_making_ros src/decision_making/ROSTask.cpp)
add_dependencies(decision_making_ros labust_mission_generate_messages_cpp)
target_link_libraries(decision_making_ros ${catkin_LIBRARIES} )

if (TinyXML2_LIBRARY)
  include_directories(${TinyXML2_INCLUDE_DIR})

  add_executable(mission_execution src/mission_execution.cpp)
  add_dependencies(mission_execution labust_mission_generate_messages_cpp)
  target_link_libraries(mission_execution ${catkin_LIBRARIES} ${TinyXML2_LIBRARY} decision_making_ros)

  add_executable(mission_parser src/mission_parser.cpp)
  target_link_libraries(mission_parser ${catkin_LIBRARIES} ${TinyXML2_LIBRARY})

  add_executable(data_event_manager src/data_event_manager.cpp)
  target_link_libraries(data_event_manager ${catkin_LIBRARIES} ${TinyXML2_LIBRARY})

  add_executable(commander src/commander.cpp)
  target_link_libraries(commander ${catkin_LIBRARIES} ${TinyXML2_LIBRARY})
else()
  message("WARNING" "TinyXML2 library is missing so some mission packages will not be built.")
endif()

## Install
install(TARGETS decision_making_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (TinyXML2_LIBRARY)
  install(TARGETS mission_execution mission_parser data_event_manager commander
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

## Mark cpp header files for installation
install(DIRECTORY include/labust include/labust_mission
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
