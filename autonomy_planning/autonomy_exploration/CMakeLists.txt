# Copyright 2026 The Openbot Authors
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

cmake_minimum_required(VERSION 3.14)
project(autonomy_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy REQUIRED)
find_package(autonomy_ros REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${autonomy_DIR}/cmake/modules")
if(NOT TARGET TinyXML2::TinyXML2)
  list(APPEND CMAKE_MODULE_PATH
    "${autonomy_DIR}/modules"
  )
  find_package(TinyXML2 REQUIRED)
endif()

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

set(DEPENDENCIES
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

add_executable(exploration_node
  src/exploration_node.cpp
  src/exploration_node_main.cpp
  src/conversions.cpp
)
target_include_directories(exploration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(exploration_node ${DEPENDENCIES} autonomy_ros)
target_link_libraries(exploration_node autonomy::autonomy)

set(_AUTONOMY_BUNDLED_LIB_RPATH "\$ORIGIN/../../../autonomy/lib")
set_target_properties(exploration_node PROPERTIES
  INSTALL_RPATH "${_AUTONOMY_BUNDLED_LIB_RPATH}"
  BUILD_WITH_INSTALL_RPATH FALSE
)

install(TARGETS exploration_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
