# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.14)
project(autonomy_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy REQUIRED)
find_package(autonomy_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(_AUTODRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../autonomy/autodriver")
if(NOT TARGET autodriver)
  set(AUTODRIVER_BUILD_TEST OFF CACHE BOOL "" FORCE)
  set(AUTODRIVER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(AUTODRIVER_BUILD_AUTONOMY_BRIDGE OFF CACHE BOOL "" FORCE)
  add_subdirectory(
    ${_AUTODRIVER_ROOT}
    ${CMAKE_CURRENT_BINARY_DIR}/autodriver EXCLUDE_FROM_ALL)
endif()

set(_AUTONOMY_BUNDLED_LIB_RPATH "\$ORIGIN/../../../autonomy/lib")

add_executable(driver_hub_node
  src/driver_hub_node.cpp
  src/ros_sensor_drivers.cpp
  src/ros_snapshot_publisher.cpp
  ${_AUTODRIVER_ROOT}/autodriver/bridge/autonomy/hub_config_loader.cpp
  ${_AUTODRIVER_ROOT}/autodriver/bridge/autonomy/sample_converter.cpp
)
target_include_directories(driver_hub_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${_AUTODRIVER_ROOT}/autodriver>
)
ament_target_dependencies(driver_hub_node
  rclcpp
  sensor_msgs
  nav_msgs
  autonomy_ros
)
target_link_libraries(driver_hub_node
  autodriver
  autonomy::autonomy
)
set_target_properties(driver_hub_node PROPERTIES
  INSTALL_RPATH "${_AUTONOMY_BUNDLED_LIB_RPATH}"
  BUILD_WITH_INSTALL_RPATH FALSE
)

install(TARGETS driver_hub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
