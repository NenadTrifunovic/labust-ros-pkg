cmake_minimum_required(VERSION 2.8.3)
project(labust_allocation)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  auv_msgs
  labust_msgs
  snippets
  roscpp
  cmake_modules
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(CGAL)

## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES labust_allocation
  CATKIN_DEPENDS auv_msgs roscpp std_msgs pluginlib
)

## Build
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(ALLOCATION_SRC src/labust_allocation/thruster_configuration.cpp
   src/labust_allocation/x2d_adaptive.cpp
   src/labust_allocation/x2d_vp.cpp)

if (${CGAL_FOUND})
	include(${CGAL_USE_FILE})
	include_directories(${CGAL_INCLUDE_DIR})
 	set(ALLOCATION_SRC ${ALLOCATION_SRC} src/labust_allocation/x2d_vp_qp.cpp)
        add_executable(qp_solver_test src/test/qp_solver.cpp)
	target_link_libraries(qp_solver_test ${catkin_LIBRARIES} ${CGAL_LIBRARIES})
else (${CGAL_FOUND})
        message("Skipping build of QP allocation due to missing CGAL library.")
endif()

add_library(labust_allocation ${ALLOCATION_SRC})
target_link_libraries(labust_allocation ${catkin_LIBRARIES})

add_executable(allocation_node src/allocation_node/allocation_node.cpp)
target_link_libraries(allocation_node ${catkin_LIBRARIES})

## Documentation
add_custom_target(docs_${PROJECT_NAME} 
  rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/docs/${PROJECT_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}
)

## Install
install(TARGETS labust_allocation allocation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/labust
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES allocation_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
