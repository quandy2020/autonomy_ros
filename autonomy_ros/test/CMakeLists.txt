# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");

# Navigation arbitration is covered via TaskManager integration tests when added.

add_executable(test_planner test_planner.cpp)
target_link_libraries(test_planner autonomy_core)
ament_target_dependencies(test_planner
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)
install(TARGETS test_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_controller
  test_controller.cpp
  path_generator.cpp
)
target_include_directories(test_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_controller autonomy_core)
ament_target_dependencies(test_controller
  rclcpp
  nav_msgs
  geometry_msgs
)
install(TARGETS test_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
