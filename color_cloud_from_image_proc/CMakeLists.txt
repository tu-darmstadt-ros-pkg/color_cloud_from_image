cmake_minimum_required(VERSION 2.8.3)
project(color_cloud_from_image_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kalibr_camera_loader
  nodelet
  pcl_ros
  tf
  tf_conversions
  #aslam_cameras
  #aslam_time
  #sm_boost
  #sm_property_tree
  #sm_eigen
  #sm_kinematics
  #sm_opencv
  #sm_logging
  #geometry_msgs
  robot_self_filter
)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
    message(STATUS "Found assimp v3")
  else()
    message(STATUS "Found assimp v2")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES color_cloud_from_image_proc
  CATKIN_DEPENDS kalibr_camera_loader nodelet pcl_ros tf tf_conversions
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(color_cloud_from_image_proc
  include/${PROJECT_NAME}/color_cloud_from_image.h
  src/color_cloud_from_image.cpp
  src/color_cloud_from_image_nodelet.cpp
)

target_link_libraries(color_cloud_from_image_proc
  ${catkin_LIBRARIES}
)
add_dependencies(color_cloud_from_image_proc ${catkin_EXPORTED_TARGETS})


add_executable(color_cloud_from_image_node src/color_cloud_from_image_node.cpp)
target_link_libraries(color_cloud_from_image_node
  ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS color_cloud_from_image_node color_cloud_from_image_proc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
    launch
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
