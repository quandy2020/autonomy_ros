cmake_minimum_required(VERSION 3.8)
project(autonomy_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(autonomy REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_ros REQUIRED)

include(FindPkgConfig)

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
)

set(executable_name autonomy_node)
add_executable(${executable_name}
  src/main.cpp
  src/autonomy_node.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} autonomy)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
