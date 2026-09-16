cmake_minimum_required(VERSION 3.8)
project(nav2_orca_controller)

find_package(ament_cmake REQUIRED)

# Fetch RVO2 from GitHub
include(FetchContent)
FetchContent_Declare(
  rvo2
  GIT_REPOSITORY https://github.com/snape/RVO2.git
  GIT_TAG v2.0.2
)
FetchContent_MakeAvailable(rvo2)

set(dependencies_pkgs
  rclcpp
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_geometry_msgs
  tf2_ros
  diagnostic_msgs
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

include_directories(
  include
)

add_library(orca_controller SHARED
  src/orca_controller.cpp
  src/obstacle_clusterer.cpp
)

target_include_directories(orca_controller PUBLIC include)
target_include_directories(orca_controller PRIVATE ${rvo2_SOURCE_DIR}/src)

target_link_libraries(orca_controller RVO)

ament_target_dependencies(orca_controller ${dependencies_pkgs})

install(TARGETS orca_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(orca_controller)
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core orca_plugin.xml)

ament_package()
