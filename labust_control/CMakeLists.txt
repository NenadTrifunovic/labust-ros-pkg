cmake_minimum_required(VERSION 2.8.3)
project(labust_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  auv_msgs
  navcon_msgs
  sensor_msgs
  misc_msgs
  dynamic_reconfigure
  roscpp
  snippets
  tf2_ros
)

find_package(rosdoc_lite)

## Generate dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/TDOASSControl.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES pidtoolkit2
   CATKIN_DEPENDS auv_msgs roscpp snippets navcon_msgs sensor_msgs
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include src/esctoolkit)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Main library
add_library(pidtoolkit2
   src/pidtoolkit/PIDBase.c
   src/pidtoolkit/PIFFController.c
   src/pidtoolkit/PSatDController.c
   src/pidtoolkit/IPFFController.c
   src/pidtoolkit/PIDFFController.c
   src/pidtoolkit/IPDFFController.c
)

set_source_files_properties(
   src/pidtoolkit/PIDBase.c
   src/pidtoolkit/PIFFController.c
   src/pidtoolkit/PSatDController.c 
   src/pidtoolkit/IPFFController.c
   src/pidtoolkit/PIDFFController.c
   src/pidtoolkit/IPDFFController.c PROPERTIES LANGUAGE CXX)

add_library(esctoolkit
   src/esctoolkit/esc_classic.cpp
)

## Manual control 
add_executable(manual_control src/manual_control/manual_control.cpp)
add_dependencies(manual_control auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(manual_control ${catkin_LIBRARIES})
add_executable(lauv_manual src/LAUVManual.cpp)
add_dependencies(lauv_manual auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(lauv_manual ${catkin_LIBRARIES})
add_executable(nu_manual src/NuManual.cpp)
add_dependencies(nu_manual auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(nu_manual ${catkin_LIBRARIES})
add_executable(ref_manual src/RefManual.cpp)
add_dependencies(ref_manual auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(ref_manual ${catkin_LIBRARIES})
 
## Velocity control
add_executable(velocity_control	src/VelocityControl.cpp)
add_dependencies(velocity_control auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(velocity_control pidtoolkit2 ${catkin_LIBRARIES})

## Position control
add_executable(fadp_controller2 src/FADPControl.cpp)
add_dependencies(fadp_controller2 auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(fadp_controller2 pidtoolkit2 ${catkin_LIBRARIES})
add_executable(fadp_3d_controller src/FADP_3DControl.cpp)
add_dependencies(fadp_3d_controller auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(fadp_3d_controller pidtoolkit2 ${catkin_LIBRARIES})
add_executable(hdg_controller2 src/HDGControl.cpp)
add_dependencies(hdg_controller2 auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(hdg_controller2 pidtoolkit2 ${catkin_LIBRARIES})
add_executable(alt_controller2 src/ALTControl.cpp)
add_dependencies(alt_controller2 auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(alt_controller2 pidtoolkit2 ${catkin_LIBRARIES})
add_executable(depth_controller2 src/DepthControl.cpp)
add_dependencies(depth_controller2 auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(depth_controller2 pidtoolkit2 ${catkin_LIBRARIES})
add_executable(pitch_controller src/PitchControl.cpp)
add_dependencies(pitch_controller auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(pitch_controller pidtoolkit2 ${catkin_LIBRARIES})

## Guidance control
add_executable(vt_controller src/virtual_target/vt_control.cpp)
add_dependencies(vt_controller auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(vt_controller pidtoolkit2 ${catkin_LIBRARIES})
add_executable(ualf_controller src/line_following/UALFControl.cpp)
add_dependencies(ualf_controller auv_msgs_gencpp navcon_msgs_gencpp)
target_link_libraries(ualf_controller pidtoolkit2 ${catkin_LIBRARIES})

## Extremum seeking control
#add_executable(esc_uv src/esc/ESCControlClassic_UV.cpp src/esctoolkit/esc_classic.cpp)
#add_dependencies(esc_uv auv_msgs_gencpp navcon_msgs_gencpp)
#target_link_libraries(esc_uv  ${catkin_LIBRARIES})
#add_executable(esc_ekf_uv src/esc/ESControlEKF_UV.cpp src/esctoolkit/esc_ekf_grad.cpp)
#add_dependencies(esc_ekf_uv auv_msgs_gencpp navcon_msgs_gencpp)
#target_link_libraries(esc_ekf_uv  ${catkin_LIBRARIES})
#add_executable(esc_ekf_model_uv src/esc/ESControlEKF_MODEL_UV.cpp src/esctoolkit/esc_ekf_grad_model.cpp)
#add_dependencies(esc_ekf_model_uv auv_msgs_gencpp navcon_msgs_gencpp)
#target_link_libraries(esc_ekf_model_uv  ${catkin_LIBRARIES})
#add_executable(esc_bounded_uv src/esc/ESControlBounded_UV.cpp src/esctoolkit/esc_bounded.cpp)
#add_dependencies(esc_bounded_uv auv_msgs_gencpp navcon_msgs_gencpp)
#target_link_libraries(esc_bounded_uv  ${catkin_LIBRARIES})

## TDOASS node
add_executable(tdoass_control src/tdoass/tdoass_control.cpp)
#add_dependencies(tdoass_control)
target_link_libraries(tdoass_control ${catkin_LIBRARIES} esctoolkit)

## Tools
add_executable(async_merger src/AsyncMerger.cpp)
add_dependencies(async_merger auv_msgs_gencpp)
target_link_libraries(async_merger ${catkin_LIBRARIES})
add_executable(navstsreq_navsts	src/navstsreq_navsts.cpp)
add_dependencies(navstsreq_navsts auv_msgs_gencpp)
target_link_libraries(navstsreq_navsts ${catkin_LIBRARIES})
add_executable(diver_navsts	src/diver_navsts.cpp)
add_dependencies(diver_navsts auv_msgs_gencpp)
target_link_libraries(diver_navsts ${catkin_LIBRARIES})

###################
## Documentation ##
###################
if (BUILD_ROS_DOCS)
    if (rosdoc_lite_FOUND)
     set(doc_files mainpage.dox)   
     add_custom_target(${PROJECT_NAME}_docs ALL rosdoc_lite -o 
         ${CMAKE_BINARY_DIR}/doc/${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} DEPENDS ${doc_files})
     endif()
endif()

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
# install(TARGETS labust_control labust_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
