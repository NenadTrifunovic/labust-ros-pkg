cmake_minimum_required(VERSION 2.8.3)
project(labust_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diagnostic_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  labust_diagnostics
  CATKIN_DEPENDS std_msgs sensor_msgs diagnostic_msgs rospy roscpp
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Main library
#file(GLOB SRC src/BatteryDiagnostics.cpp src/ThrusterDiagnostics.cpp)
#file(GLOB HPP include/labust/diagnostics/{BatteryDiagnostics.hpp include/labust/diagnostics/ThrusterDiagnostics.hpp)
file(GLOB SRC src/StatusHandler.cpp)
file(GLOB HPP)
add_library(labust_diagnostics ${SRC} ${HPP})
target_link_libraries(labust_diagnostics ${catkin_LIBRARIES})

## Diagnostic nodes
#add_executable(system_diagnostics_node src/SystemDiagnosticsNode.cpp)
#target_link_libraries(system_diagnostics_node labust_diagnostics ${catkin_LIBRARIES})

add_executable(diagnostic_node src/diagnostic_node.cpp)
target_link_libraries(diagnostic_node labust_diagnostics ${catkin_LIBRARIES})

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
# install(TARGETS image_processing_util image_processing_util_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_processing_util.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
