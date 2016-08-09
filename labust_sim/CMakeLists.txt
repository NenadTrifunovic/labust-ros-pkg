cmake_minimum_required(VERSION 2.8.3)
project(labust_sim)

find_package(catkin REQUIRED COMPONENTS 
    cmake_modules
    auv_msgs 
    nav_msgs
    underwater_msgs
    navcon_msgs
    roscpp 
    rospy 
    tf2_ros 
    snippets)

## System dependencies 
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES labust_sim
   CATKIN_DEPENDS auv_msgs roscpp rospy tf2_ros nav_msgs snippets nav_msgs underwater_msgs navcon_msgs
   DEPENDS boost eigen
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(include
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Main simulation library
file(GLOB SRC src/labust_sim/*.cpp)
file(GLOB HPP include/labust/simulation/*.hpp include/labust/simulation/*.h) 
add_library(labust_sim ${SRC} ${HPP})
add_dependencies(labust_sim auv_msgs_gencpp)
target_link_libraries(labust_sim ${Boost_RANDOM_LIBRARY})

## Main simulator node
file(GLOB SRC src/uvsim/*.cpp)
file(GLOB HPP include/labust/ros/*.hpp) 
add_executable(uvsim ${SRC} ${HPP})
target_link_libraries(uvsim ${catkin_LIBRARIES} labust_sim ${Boost_RANDOM_LIBRARY})

## Simulated sensors
add_executable(imu_sim src/sim_sensors/ImuSim.cpp)
target_link_libraries(imu_sim ${catkin_LIBRARIES})

add_executable(dvl_sim src/sim_sensors/DvlSim.cpp)
target_link_libraries(dvl_sim ${catkin_LIBRARIES} ${Boost_RANDOM_LIBRARY})

add_executable(gps_sim src/sim_sensors/GPSSim.cpp)
target_link_libraries(gps_sim ${catkin_LIBRARIES} Geographic ${Boost_RANDOM_LIBRARY})

add_executable(usbl_sim src/data_sim/USBLSim.cpp)
add_dependencies(usbl_sim auv_msgs_gencpp underwater_msgs_gencpp)
target_link_libraries(usbl_sim ${catkin_LIBRARIES})

add_executable(sonar_sim src/sim_sensors/SonarSim.cpp)
add_dependencies(sonar_sim navcon_msgs_gencpp)
target_link_libraries(sonar_sim ${catkin_LIBRARIES} ${Boost_RANDOM_LIBRARY})

## Diver simulator
add_executable(diver_sim src/diver_sim/diver_sim.cpp)
add_dependencies(diver_sim auv_msgs_gencpp)
target_link_libraries(diver_sim labust_sim ${catkin_LIBRARIES})

add_executable(virtual_diver src/diver_sim/virtual_diver.cpp)
add_dependencies(virtual_diver auv_msgs_gencpp)
target_link_libraries(virtual_diver ${catkin_LIBRARIES})

## Tools
add_executable(navsts2odom src/navsts2odom/navsts2odom.cpp)
add_dependencies(navsts2odom auv_msgs_gencpp)
target_link_libraries(navsts2odom ${catkin_LIBRARIES})

add_executable(uvsim_timing src/test/uvsim_timing.cpp)
target_link_libraries(uvsim_timing labust_sim ${catkin_LIBRARIES})

add_executable(trajectory_generator src/labust_sim/trajectory_generator.cpp)
add_dependencies(trajectory_generator auv_msgs_gencpp underwater_msgs_gencpp ${underwater_msgs_EXPORTED_TARGETS})
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS labust_sim labust_sim_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
