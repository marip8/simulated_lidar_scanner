cmake_minimum_required(VERSION 2.8.3)
project(simulated_lidar_scanner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pcl_ros
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    interactive_markers
    pcl_ros
    tf2_eigen
    tf2_ros
    urdf
    visualization_msgs
  DEPENDS
    VTK
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
 src/synthetic_lidar_scanner/vtkLidarScanner.cxx
 src/synthetic_lidar_scanner/vtkRay.cxx
 src/synthetic_lidar_scanner/vtkLidarPoint.cxx
 src/lidar_scanner_simulator.cpp
 src/scene_builder.cpp
)
target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
 ${VTK_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/lidar_scanner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(scanner_relocator
  src/scanner_relocator.cpp
)
target_link_libraries(scanner_relocator
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    scanner_relocator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY demo launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/simulated_lidar_scanner.test)
endif()
