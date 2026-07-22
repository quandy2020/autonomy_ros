# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(autonomy_pedestrian)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(SOCIAL_LAYER_LIB nav2_social_costmap_plugin_core)

set(EXTERNAL_DEPS
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  derived_object_msgs
  pedsim_msgs
  ros_tools
  tf2_ros
)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(pkg IN LISTS EXTERNAL_DEPS)
  find_package(${pkg} REQUIRED)
endforeach()

# Vendored header-only RapidXML (was asr_rapidxml).
add_library(rapidxml INTERFACE)
target_include_directories(rapidxml INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/rapidxml/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/third_party/rapidxml>"
)

# Vendored pedsim core (was pedsim_original).
add_library(pedsim_original SHARED
  src/pedsim/ped_agent.cpp
  src/pedsim/ped_angle.cpp
  src/pedsim/ped_obstacle.cpp
  src/pedsim/ped_scene.cpp
  src/pedsim/ped_tree.cpp
  src/pedsim/ped_vector.cpp
  src/pedsim/ped_waypoint.cpp
)
target_include_directories(pedsim_original PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(pedestrians SHARED
  src/pedestrians/binomial_pedestrian.cpp
  src/pedestrians/gaussian_pedestrian.cpp
  src/pedestrians/pedestrian.cpp
  src/pedestrians/random_gaussian_pedestrian.cpp
  src/pedestrians/social_forces_pedestrian.cpp
)
target_include_directories(pedestrians PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(pedestrians Eigen3::Eigen rapidxml)
ament_target_dependencies(pedestrians ros_tools)

add_library(pedestrian_simulator_core SHARED
  src/simulator/pedestrian_simulator.cpp
  src/simulator/pedsim_manager.cpp
  src/simulator/xml_reader.cpp
  src/simulator/spawn_randomizer.cpp
  src/simulator/configuration.cpp
)
target_include_directories(pedestrian_simulator_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(pedestrian_simulator_core
  pedestrians
  pedsim_original
  Eigen3::Eigen
  rapidxml
)
ament_target_dependencies(pedestrian_simulator_core ${EXTERNAL_DEPS})

add_executable(pedestrian_simulator_node src/simulator/ros2_pedestrian_simulator.cpp)
target_link_libraries(pedestrian_simulator_node pedestrian_simulator_core)
ament_target_dependencies(pedestrian_simulator_node ${EXTERNAL_DEPS})

add_library(${SOCIAL_LAYER_LIB} SHARED
  src/social_costmap/social_layer.cpp
)
target_include_directories(${SOCIAL_LAYER_LIB} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${SOCIAL_LAYER_LIB}
  rclcpp
  nav2_costmap_2d
  pluginlib
  pedsim_msgs
  nav_msgs
)

install(TARGETS
  rapidxml
  pedsim_original
  pedestrians
  pedestrian_simulator_core
  ${SOCIAL_LAYER_LIB}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS pedestrian_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY third_party/rapidxml/include/
  DESTINATION include/${PROJECT_NAME}/third_party/rapidxml
)
install(DIRECTORY launch config scenarios scripts rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(FILES social_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d social_layer.xml)

install(PROGRAMS
  scripts/pedestrian_demo.py
  scripts/keyboard_teleop.py
  scripts/pedestrian_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${EXTERNAL_DEPS} Eigen3 nav2_costmap_2d pluginlib)
ament_package()
