cmake_minimum_required(VERSION 2.8.3)
project(labust_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  cmake_modules  
  auv_msgs
  labust_msgs
  sensor_msgs
  dynamic_reconfigure
  snippets
  tf2_ros
)

#find_package(rosdoc_lite)

## Generate dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/VelCon.cfg
)

## Catkin specific configuration
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES pidtoolkit2
   CATKIN_DEPENDS auv_msgs roscpp snippets labust_msgs sensor_msgs
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main library
add_library(pidtoolkit2
   src/pidtoolkit/PIDBase.c
   src/pidtoolkit/PIFFController.c
   src/pidtoolkit/PFFController.c   
   src/pidtoolkit/PSatDController.c
   src/pidtoolkit/IPFFController.c
   src/pidtoolkit/PIDFFController.c
   src/pidtoolkit/IPDFFController.c
)

set_source_files_properties(
   src/pidtoolkit/PIDBase.c
   src/pidtoolkit/PIFFController.c
   src/pidtoolkit/PFFController.c   
   src/pidtoolkit/PSatDController.c 
   src/pidtoolkit/IPFFController.c
   src/pidtoolkit/PIDFFController.c
   src/pidtoolkit/IPDFFController.c PROPERTIES LANGUAGE CXX)

# Manual control 
add_executable(manual_control src/manual_control/manual_control.cpp)
target_link_libraries(manual_control ${catkin_LIBRARIES})

add_executable(lauv_manual src/LAUVManual.cpp)
target_link_libraries(lauv_manual ${catkin_LIBRARIES})

add_executable(nu_manual src/NuManual.cpp)
target_link_libraries(nu_manual ${catkin_LIBRARIES})

add_executable(ref_manual src/RefManual.cpp)
target_link_libraries(ref_manual ${catkin_LIBRARIES})
 
# Velocity control
add_executable(velocity_control	src/VelocityControl.cpp)
add_dependencies(velocity_control labust_control_gencfg)
target_link_libraries(velocity_control pidtoolkit2 ${catkin_LIBRARIES})

# Position control
add_executable(fadp_controller2 src/FADPControl.cpp)
target_link_libraries(fadp_controller2 pidtoolkit2 ${catkin_LIBRARIES})

add_executable(fadp_3d_controller src/FADP_3DControl.cpp)
target_link_libraries(fadp_3d_controller pidtoolkit2 ${catkin_LIBRARIES})

add_executable(hdg_controller2 src/HDGControl.cpp)
target_link_libraries(hdg_controller2 pidtoolkit2 ${catkin_LIBRARIES})

add_executable(alt_controller2 src/ALTControl.cpp)
target_link_libraries(alt_controller2 pidtoolkit2 ${catkin_LIBRARIES})

add_executable(depth_controller2 src/DepthControl.cpp)
target_link_libraries(depth_controller2 pidtoolkit2 ${catkin_LIBRARIES})

add_executable(pitch_controller src/PitchControl.cpp)
target_link_libraries(pitch_controller pidtoolkit2 ${catkin_LIBRARIES})

# Guidance control
add_executable(vt_controller src/virtual_target/vt_control.cpp)
target_link_libraries(vt_controller pidtoolkit2 ${catkin_LIBRARIES})

add_executable(ualf_controller src/line_following/UALFControl.cpp)
target_link_libraries(ualf_controller pidtoolkit2 ${catkin_LIBRARIES})

# Tools
add_executable(async_merger src/AsyncMerger.cpp)
target_link_libraries(async_merger ${catkin_LIBRARIES})

add_executable(navstsreq_navsts	src/navstsreq_navsts.cpp)
target_link_libraries(navstsreq_navsts ${catkin_LIBRARIES})

add_executable(diver_navsts	src/diver_navsts.cpp)
target_link_libraries(diver_navsts ${catkin_LIBRARIES})

## Documentation
#if (BUILD_ROS_DOCS)
#    if (rosdoc_lite_FOUND)
#     set(doc_files mainpage.dox)   
#     add_custom_target(${PROJECT_NAME}_docs ALL rosdoc_lite -o 
#         ${CMAKE_BINARY_DIR}/doc/${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} DEPENDS ${doc_files})
#     endif()
#endif()

## Install
install(TARGETS pidtoolkit2
  manual_control lauv_manual nu_manual ref_manual velocity_control
  fadp_controller2 fadp_3d_controller hdg_controller2 alt_controller2 depth_controller2 pitch_controller
  vt_controller ualf_controller
  async_merger navstsreq_navsts diver_navsts
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/labust
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
