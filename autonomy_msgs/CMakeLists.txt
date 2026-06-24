cmake_minimum_required(VERSION 3.8)
project(autonomy_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  msg/TaskType.msg
  msg/TaskState.msg
  msg/Error.msg
  msg/Waypoint.msg
  msg/WaypointStatus.msg
  msg/TaskStatus.msg
  msg/Event.msg
  msg/GraphNode.msg
  msg/GraphEdge.msg
  msg/GraphFace.msg
  msg/Graph.msg
)

set(srv_files
  srv/CancelTask.srv
  srv/GetTaskStatus.srv
  srv/PauseTask.srv
  srv/ResumeTask.srv
  srv/TriggerEmergencyStop.srv
  srv/SetInitialPose.srv
)

set(action_files
  action/NavigatePose.action
  action/NavigateThrough.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
)

set(CMAKE_AUTOMOC ON)

find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

set(RVIZ_LIB autonomy_msgs_rviz)

add_library(${RVIZ_LIB} SHARED
  src/graph_display.cpp
  include/autonomy_msgs/graph_display.hpp
)

target_include_directories(${RVIZ_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

ament_target_dependencies(${RVIZ_LIB}
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_rendering
  std_msgs
)

target_link_libraries(${RVIZ_LIB}
  rviz_common::rviz_common
  Qt5::Widgets
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${RVIZ_LIB} "${cpp_typesupport_target}")

target_compile_definitions(${RVIZ_LIB} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${RVIZ_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
