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
project(autonomy_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy REQUIRED)
find_package(autonomy_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# autonomy::autonomy PUBLIC interface (re-find when config lags behind rebuild)
find_package(nlohmann_json QUIET)
find_package(foxglove-sdk CONFIG QUIET)
find_package(ZLIB QUIET)

list(APPEND CMAKE_MODULE_PATH "${autonomy_DIR}/cmake/modules")
if(NOT TARGET TinyXML2::TinyXML2)
  list(APPEND CMAKE_MODULE_PATH "${autonomy_DIR}/modules")
  find_package(TinyXML2 REQUIRED)
endif()

set(_AUTONOMY_BUNDLED_LIB_RPATH "\$ORIGIN/../../../autonomy/lib")

set(_SIM_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  autonomy_ros
)

add_executable(planner_sim_node
  src/planner_sim_node.cpp
  src/planner_sim_utils.cpp
)
target_include_directories(planner_sim_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(planner_sim_node ${_SIM_DEPS})
target_link_libraries(planner_sim_node autonomy::autonomy)
set_target_properties(planner_sim_node PROPERTIES
  INSTALL_RPATH "${_AUTONOMY_BUNDLED_LIB_RPATH}"
  BUILD_WITH_INSTALL_RPATH FALSE
)

install(TARGETS planner_sim_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/obstacle_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(_planner_tests
    test/test_planner_options.cpp
    test/test_navfn_planner.cpp
    test/test_dijkstra_planner.cpp
    test/test_theta_star_planner.cpp
    test/test_path_simplifier.cpp
  )

  foreach(_src IN LISTS _planner_tests)
    get_filename_component(_name ${_src} NAME_WE)
    ament_add_gtest(${_name} ${_src})
    target_include_directories(${_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    ament_target_dependencies(${_name} rclcpp)
    target_link_libraries(${_name} autonomy::autonomy)
    set_target_properties(${_name} PROPERTIES
      INSTALL_RPATH "${_AUTONOMY_BUNDLED_LIB_RPATH}"
      BUILD_WITH_INSTALL_RPATH FALSE
    )
  endforeach()
endif()

ament_package()
