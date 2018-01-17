cmake_minimum_required(VERSION 2.8.3)
project(labust_primitives)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  auv_msgs
  labust_msgs
  roscpp
  tf2_ros
  snippets
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
# LIBRARIES labust_primitives
  CATKIN_DEPENDS actionlib actionlib_msgs auv_msgs labust_msgs roscpp tf2_ros snippets
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Primitives
add_executable(go2point src/go2point.cpp)
add_dependencies(go2point auv_msgs_gencpp labust_msgs_gencpp)
target_link_libraries(go2point ${catkin_LIBRARIES})

add_executable(course_keeping src/course_keeping.cpp)
add_dependencies(course_keeping auv_msgs_gencpp labust_msgs_gencpp)
target_link_libraries(course_keeping ${catkin_LIBRARIES})

add_executable(dynamic_positioning src/dynamic_positioning.cpp)
add_dependencies(dynamic_positioning auv_msgs_gencpp labust_msgs_gencpp)
target_link_libraries(dynamic_positioning ${catkin_LIBRARIES})

add_executable(track_diver src/track_diver.cpp)
add_dependencies(track_diver auv_msgs_gencpp labust_msgs_gencpp)
target_link_libraries(track_diver ${catkin_LIBRARIES})

add_executable(docking src/docking.cpp)
add_dependencies(docking auv_msgs_gencpp labust_msgs_gencpp)
target_link_libraries(docking ${catkin_LIBRARIES})

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
# install(TARGETS labust_primitives labust_primitives_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_primitives.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
