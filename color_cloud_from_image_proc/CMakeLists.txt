cmake_minimum_required(VERSION 3.8)
project(color_cloud_from_image_proc)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
set(DEPENDENCIES
  extended_camera_loader
  pcl_ros
  tf2
  tf2_ros
  tf2_sensor_msgs
  #eigen_conversions
  #robot_self_filter
  cv_bridge
  message_filters
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

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

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/color_cloud_from_image.h
  src/color_cloud_from_image.cpp
  #src/color_cloud_from_image_node.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# composable node
rclcpp_components_register_nodes(${PROJECT_NAME} PLUGIN "color_cloud_from_image::ColorCloudFromImage" EXECUTABLE color_cloud_from_image_component)

# target_link_libraries(color_cloud_from_image_proc
#   ${catkin_LIBRARIES}
# )
# add_dependencies(color_cloud_from_image_proc ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_standalone_node src/color_cloud_from_image_standalone_node.cpp)
#ament_target_dependencies(${PROJECT_NAME}_standalone_node PUBLIC ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_standalone_node
  ${PROJECT_NAME}
)

# Mark executables and/or libraries for installation
ament_export_targets(export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_standalone_node 
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
