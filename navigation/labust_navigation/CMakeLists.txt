cmake_minimum_required(VERSION 2.8.3)
project(labust_navigation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  auv_msgs
  roscpp
  snippets
  sensor_msgs
  labust_msgs
  geometry_msgs
  tf2_ros
  underwater_msgs
  labust_diagnostics
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)
find_package(GeographicLib REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES labust_navigation
  CATKIN_DEPENDS auv_msgs roscpp snippets underwater_msgs geometry_msgs labust_msgs sensor_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Main navigation library
add_library(labust_navigation
   src/labust_navigation/KinematicModel.cpp
   src/labust_navigation/XYModel.cpp
   src/labust_navigation/SensorHandlers.cpp
   src/labust_navigation/LDTravModelExtended.cpp
   src/labust_navigation/EKF_3D_USBLModel.cpp 
   #src/labust_navigation/TwoVehicleLocalizationModel.cpp 
   #src/labust_navigation/SimpleRelativeLocalizationModel.cpp   
)
#add_dependencies(labust_navigation)
target_link_libraries(labust_navigation
   ${catkin_LIBRARIES}
   ${GeographicLib_LIBRARIES}
)

## Navframe nodes
add_executable(llnode src/latlon_node/LLNode.cpp)
target_link_libraries(llnode ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
add_executable(latlon2ned src/latlon_node/latlon2ned.cpp)
add_dependencies(latlon2ned auv_msgs_gencpp)
target_link_libraries(latlon2ned ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
add_executable(ltp_node src/latlon_node/ltp_node.cpp)
target_link_libraries(ltp_node ${catkin_LIBRARIES})
add_executable(tf_publisher_node src/nav_node/tf_publisher_node.cpp)
add_dependencies(tf_publisher_node auv_msgs_gencpp)
target_link_libraries(tf_publisher_node ${catkin_LIBRARIES})

## Filters 
add_executable(ekf3d_node src/nav_node/EKF3D.cpp)
target_link_libraries(ekf3d_node ${catkin_LIBRARIES} labust_navigation)
add_executable(ekf_3d_usbl_node src/nav_node/EKF_3D_USBL.cpp)
target_link_libraries(ekf_3d_usbl_node ${catkin_LIBRARIES} labust_navigation)
#add_executable(ekf_two_vehicle_localization_node src/nav_node/EKF_TwoVehicleLocalization.cpp)
#target_link_libraries(ekf_two_vehicle_localization_node ${catkin_LIBRARIES} labust_navigation)
#add_executable(ekf_simple_relative_localization_node src/nav_node/EKF_SimpleRelativeLocalization.cpp)
#target_link_libraries(ekf_simple_relative_localization_node ${catkin_LIBRARIES} labust_navigation)

## Tools
#add_executable(euler_quat src/test/euler_quat.cpp)
#target_link_libraries(euler_quat ${catkin_LIBRARIES} labust_navigation)
#add_executable(quat_to_euler src/test/quat_to_euler.cpp)
#add_dependencies(quat_to_euler auv_msgs_gencpp)
#target_link_libraries(quat_to_euler ${catkin_LIBRARIES})
#add_executable(static_transform_publisher src/static_transform_publisher/static_transform_publisher.cpp)
#target_link_libraries(static_transform_publisher ${catkin_LIBRARIES})
#add_executable(pressure2depth src/test/pressure2depth.cpp)
#target_link_libraries(pressure2depth ${catkin_LIBRARIES})

## Tests
#add_executable(eigen_test src/test/eigen_test.cpp)
#target_link_libraries(eigen_test ${catkin_LIBRARIES} labust_navigation)
#add_executable(decomposition_test src/test/decomposition_test.cpp)
#add_executable(tf_comp_test src/test/tf_comp_test.cpp)
#target_link_libraries(tf_comp_test ${catkin_LIBRARIES} labust_navigation)

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
# install(TARGETS labust_navigation labust_navigation_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
