# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(autonomy_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy REQUIRED)

# autonomy::autonomy links TinyXML2::TinyXML2 (autolink plugin XML parser).
if(NOT TARGET TinyXML2::TinyXML2)
  list(APPEND CMAKE_MODULE_PATH
    "${autonomy_DIR}/modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules"
  )
  find_package(TinyXML2 REQUIRED)
endif()
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

set(AUTONOMY_CORE_SOURCES
  src/conversions/detail.cpp
  src/conversions/builtin_interfaces.cpp
  src/conversions/std_msgs.cpp
  src/conversions/geometry_msgs.cpp
  src/conversions/planning_msgs.cpp
  src/conversions/map_msgs.cpp
  src/conversions/sensor_msgs.cpp
  src/conversions/visualization_msgs.cpp
  src/conversions/diagnostic_msgs.cpp
  src/conversions/trajectory_msgs.cpp
  src/conversions/vision_msgs.cpp
  src/conversions/stereo_msgs.cpp
  src/conversions/shape_msgs.cpp
  src/conversions/pcl_msgs.cpp
  src/conversions/tf2_msgs.cpp
  src/ros_autonomy_system.cpp
  src/options.cpp
  src/bridge/tf_bridge.cpp
  src/bridge/map_bridge.cpp
  src/bridge/costmap_bridge.cpp
  src/bridge/actuation_bridge.cpp
  src/bridge/sensor_bridge.cpp
  src/visualization/visualizer.cpp
  src/debug/diagnostics_publisher.cpp
  src/task/task_muxer.cpp
  src/task/task_manager.cpp
  src/command/command_interface.cpp
)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  nav_msgs
  map_msgs
  octomap_msgs
  grid_map_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  diagnostic_msgs
  trajectory_msgs
  vision_msgs
  stereo_msgs
  shape_msgs
  pcl_msgs
  autonomy_msgs
  builtin_interfaces
  tf2_msgs
  std_srvs
  std_msgs
)

add_library(autonomy_core ${AUTONOMY_CORE_SOURCES})
target_include_directories(autonomy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(autonomy_core ${DEPENDENCIES} )
target_link_libraries(autonomy_core autonomy::autonomy)

add_executable(autonomy_node src/main.cpp)
target_link_libraries(autonomy_node autonomy_core)
ament_target_dependencies(autonomy_node rclcpp)

install(TARGETS autonomy_core autonomy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz config docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/autonomy_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/examples
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(autonomy_core)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
