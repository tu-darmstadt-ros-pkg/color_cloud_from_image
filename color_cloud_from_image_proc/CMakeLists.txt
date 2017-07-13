cmake_minimum_required(VERSION 2.8.3)
project(color_cloud_from_image_proc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_model_loader
  nodelet
  pcl_ros
  tf
  tf_conversions
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES color_cloud_from_image_proc
  CATKIN_DEPENDS camera_model_loader nodelet pcl_ros tf tf_conversions
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


add_executable(color_cloud_from_image_proc_node src/color_cloud_from_image_node.cpp)
target_link_libraries(color_cloud_from_image_proc_node
  ${catkin_LIBRARIES}
)
