cmake_minimum_required(VERSION 3.8)
project(autonomy_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(AUTONOMY_CORE_SOURCES
  src/autonomy_node.cpp
  src/map/map_manager.cpp
  src/planner/planner.cpp
  src/controller/controller.cpp
  src/visualization/visualizer.cpp
  src/task/task_muxer.cpp
  src/task/task_manager.cpp
  src/command/command_interface.cpp
)

add_library(autonomy_core ${AUTONOMY_CORE_SOURCES})
target_include_directories(autonomy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(autonomy_core
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  visualization_msgs
  autonomy_msgs
  builtin_interfaces
)

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

install(DIRECTORY launch config rviz configs worlds urdf models docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_muxer test/test_task_muxer.cpp)
  if(TARGET test_task_muxer)
    target_link_libraries(test_task_muxer autonomy_core)
  endif()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(autonomy_core)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  visualization_msgs
  autonomy_msgs
  builtin_interfaces
)
ament_package()
