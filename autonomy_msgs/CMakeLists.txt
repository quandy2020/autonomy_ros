cmake_minimum_required(VERSION 3.8)
project(autonomy_msgs)

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
  msg/FollowTarget.msg
  msg/DockStation.msg
  msg/ExhibitPoint.msg
  msg/BatteryStatus.msg
  msg/TaskStatus.msg
  msg/Event.msg
)

set(srv_files
  srv/CancelTask.srv
  srv/GetTaskStatus.srv
  srv/SetTeleopMode.srv
  srv/ListDocks.srv
  srv/PauseTask.srv
  srv/ResumeTask.srv
  srv/SkipToExhibit.srv
  srv/ContinueTour.srv
  srv/TriggerEmergencyStop.srv
  srv/SetInitialPose.srv
)

set(action_files
  action/NavigatePose.action
  action/NavigateThrough.action
  action/Follow.action
  action/GuidedTour.action
  action/Dock.action
  action/Teleop.action
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
