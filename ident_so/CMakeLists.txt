cmake_minimum_required(VERSION 2.8.3)
project(ident_so)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  actionlib
  snippets
  roscpp
  navcon_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES I_SO
   CATKIN_DEPENDS auv_msgs roscpp actionlib snippets
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Main library
add_library(I_SO
   src/SOIdentification.cpp
   include/labust/control/SOIdentification.hpp
   include/labust/control/Relay.hpp
)

## Identification node
add_executable(ident_so src/IdentificationNode.cpp
    include/labust/control/IdentificationNode.hpp)
add_dependencies(ident_so auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(ident_so I_SO ${catkin_LIBRARIES})
## Tests
add_executable(test_ident src/test/test_ident.cpp)
add_dependencies(test_ident auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(test_ident ${catkin_LIBRARIES})

###################
## Documentation ##
###################
add_custom_target(docs_${PROJECT_NAME} 
  rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/docs/${PROJECT_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}
)

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
# install(TARGETS ident_so ident_so_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ident_so.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
