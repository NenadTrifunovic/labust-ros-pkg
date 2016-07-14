cmake_minimum_required(VERSION 2.8.3)
project(labust_execution)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  navcon_msgs
  std_msgs
  roscpp
  rospy
)

find_package(Boost REQUIRED COMPONENTS graph)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES labust_execution
  CATKIN_DEPENDS navcon_msgs roscpp rospy
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp executable
## Execution controller
add_executable(exec_control src/ExecControl.cpp
	src/ExecDepGraph.cpp
	src/ExecPNGraph.cpp
	src/PNController.cpp)  
add_dependencies(exec_control navcon_msgs_gencpp)
target_link_libraries(exec_control ${catkin_LIBRARIES})
## Test
add_executable(test_exec src/test/test_exec.cpp)
add_dependencies(test_exec navcon_msgs_gencpp)
target_link_libraries(test_exec ${catkin_LIBRARIES})

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
# install(TARGETS labust_execution labust_execution_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_execution.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
