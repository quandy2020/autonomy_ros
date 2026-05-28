# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# email: quandy2020@126.com
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
project(autonomy_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy REQUIRED)

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
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# source
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER SOURCES EXCLUDE REGEX ".*/main\\.cpp$")

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  nav_msgs
  map_msgs
  octomap_msgs
  grid_map_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  autonomy_msgs
  builtin_interfaces
  tf2_msgs
  std_srvs
  std_msgs
)

add_library(autonomy_core ${SOURCES})
target_include_directories(autonomy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(autonomy_core ${DEPENDENCIES})
target_link_libraries(autonomy_core autonomy::autonomy)

add_executable(autonomy_node src/main.cpp)
target_link_libraries(autonomy_node autonomy_core)
ament_target_dependencies(autonomy_node rclcpp)

install(TARGETS autonomy_core autonomy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz config docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/navigation_client.py DESTINATION lib/${PROJECT_NAME})

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
