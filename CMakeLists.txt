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
find_package(rosidl_default_generators REQUIRED)

# Process isolation: link Autolink + automsgs only (NOT libautonomy).
# Pin Protobuf to /usr/local before find_package(automsgs): torch's site-packages
# ships a conflicting google/protobuf that makes generated .pb.h abstract
# (Message::New / GetArenaForAllocation mismatch).
set(Protobuf_INCLUDE_DIR "/usr/local/include" CACHE PATH
    "Protobuf headers (must match protoc used for automsgs)" FORCE)
set(Protobuf_LIBRARY "/usr/local/lib/libprotobuf.so" CACHE FILEPATH
    "libprotobuf" FORCE)
find_program(_AUTONOMY_ROS_PROTOC NAMES protoc
  PATHS /usr/local/bin /usr/bin NO_DEFAULT_PATH)
if(_AUTONOMY_ROS_PROTOC)
  set(Protobuf_PROTOC_EXECUTABLE "${_AUTONOMY_ROS_PROTOC}" CACHE FILEPATH
      "protoc" FORCE)
endif()
find_package(Protobuf REQUIRED)
find_package(Autolink REQUIRED)
find_package(automsgs REQUIRED)
find_package(glog REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/msg/WaypointStatus.msg"
  "msgs/srv/SetInitialPose.srv"
  "msgs/srv/CancelTask.srv"
  "msgs/action/NavigateToPose.action"
  "msgs/action/NavigateThroughPoses.action"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
    action_msgs
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER SOURCES EXCLUDE REGEX ".*/main\\.cpp$")

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
  tf2_msgs
  std_msgs
  action_msgs
)

add_library(autonomy_ros_core ${SOURCES})
target_include_directories(autonomy_ros_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Prefer /usr/local protobuf over any transitive torch include.
target_include_directories(autonomy_ros_core SYSTEM BEFORE PUBLIC
  ${Protobuf_INCLUDE_DIR}
)
ament_target_dependencies(autonomy_ros_core ${DEPENDENCIES})
target_link_libraries(autonomy_ros_core
  Autolink::Autolink
  automsgs::automsgs
  protobuf::libprotobuf
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(autonomy_ros_core "${cpp_typesupport_target}")

add_executable(autonomy_ros_node src/main.cpp)
target_link_libraries(autonomy_ros_node autonomy_ros_core ${GLOG_LIBRARIES})
ament_target_dependencies(autonomy_ros_node rclcpp)

install(TARGETS autonomy_ros_core autonomy_ros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz config docs DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(autonomy_ros_core)
ament_export_dependencies(${DEPENDENCIES} rosidl_default_runtime)
ament_package()
