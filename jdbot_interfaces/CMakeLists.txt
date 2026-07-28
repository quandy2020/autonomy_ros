cmake_minimum_required(VERSION 3.8)
project(jdbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(otel_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/ArmLeftState.msg"
  "msg/ArmRightState.msg"
  "msg/BmsInfo.msg"
  "msg/FactoryStatus.msg"
  "msg/HeadState.msg"
  "msg/JointControlCmd.msg"
  "msg/JointType.msg"
  "msg/UpperBodyState.msg"
  "msg/RobotState.msg"
  "msg/FindItemInfo.msg"
  "msg/mapping/MappingStatus.msg"
  "msg/scheduler/Resource.msg"
  "msg/scheduler/TryAcquireResourceRequestResource.msg"
  "msg/mapping/MappingProcessStage.msg"
  "msg/mapping/BuildMode.msg"
  "msg/robot_event/RobotEvent.msg"
  "msg/robot_event/FallDownEvent.msg"
  "msg/robot_event/WaterEvent.msg"
  "msg/robot_event/WaveHandEvent.msg"
  "msg/robot_event/GestureEvent.msg"
  "msg/robot_event/PowerOffEvent.msg"
  "msg/RLControlType.msg"
  "msg/JointControlState.msg"
  "msg/terra/POI.msg"
  "msg/terra/Topology.msg"
  "msg/terra/TopologyNode.msg"
  "msg/terra/TopologyEdge.msg"
  "msg/terra/TopologyEdgeType.msg"
  "msg/PersonMask.msg"
  "msg/embodied/MoveJ.msg"
  "msg/embodied/MoveL.msg"
  "msg/RLObservation.msg"
  "msg/TopoNode.msg"
  "msg/TopoEdge.msg"
  "msg/LockerErrorCode.msg"
  "msg/rl_debug/JointState.msg"
  "msg/rl_debug/SystemStatus.msg"
  "msg/rl_debug/RobotPose.msg"
  "msg/rl_debug/JointGroup.msg"
  "msg/rl_debug/ImuData.msg"
  "msg/rl_debug/EkfEstimate.msg"
  "msg/rl_debug/ExtCmd.msg"
  "msg/rl_debug/Inference.msg"
  "msg/rl_debug/RlTopic.msg"
  "msg/terra/TerraStatus.msg"
  "msg/terra/TerraSLAMState.msg"
  "msg/JDR_SLAM/MapData.msg"
  "msg/JDR_SLAM/KeyFrame.msg"
  "msg/JDR_SLAM/SLAMState.msg"
  "msg/JDR_SLAM/SLAMInfo.msg"
  "msg/embodied/ArmStates.msg"
  "msg/embodied/AddCartesianWaypoint.msg"
  "msg/embodied/AddJointWaypoint.msg"
  "msg/SensorEvent.msg"
  "msg/SensorEventData.msg"
  "msg/SkateCmd.msg"
  "msg/RgbdPosCmd.msg"
  "msg/embodied/EmHeadStates.msg"
  "msg/DockIR.msg"
  "msg/embodied/ForceSensor.msg"
  "msg/embodied/PickerCmd.msg"
  "msg/embodied/Debug.msg"
  "msg/embodied/PickerStatus.msg"
  # srv
  "srv/JDR_SLAM/SaveKeyFrameDataAndGenerate3DMap.srv"
  "srv/FactoryControl.srv"
  "srv/SetChassisMode.srv"
  "srv/SetBumperMode.srv"
  "srv/SetBumperThreshold.srv"
  "srv/SetDevice.srv"
  "srv/SetHeadClearError.srv"
  "srv/SetHeadMotorStop.srv"
  "srv/SetHeadParam.srv"
  "srv/GetHeadReset.srv"
  "srv/GetHeadVersion.srv"
  "srv/SetHeadZero.srv"
  "srv/SetHeadYawZero.srv"
  "srv/SetArmLeftMode.srv"
  "srv/SetArmRightMode.srv"
  "srv/SetFindItemParam.srv"
  "srv/GetBumperThreshold.srv"
  "srv/GetChassisHeight.srv"
  "srv/GetChassisMode.srv"
  "srv/SetUpperMaxVel.srv"
  "srv/GetMcuVersion.srv"
  "srv/mapping/GetMappingStatus.srv"
  "srv/mapping/StartMapping.srv"
  "srv/mapping/EndMapping.srv"
  "srv/mapping/StartLocalization.srv"
  "srv/mapping/EndLocalization.srv"
  "srv/scheduler/TryAcquireResource.srv"
  "srv/SentAudio.srv"
  "srv/SetPushStreamParam.srv"
  "srv/FaceLogin.srv"
  "srv/terra/StartNodeMapping.srv"
  "srv/terra/StartNextMapping.srv"
  "srv/terra/LoadMap.srv"
  "srv/terra/SwitchNode.srv"
  "srv/terra/CreatePoi.srv"
  "srv/terra/GetMapInfo.srv"
  "srv/terra/ListMaps.srv"
  "srv/terra/ListPoi.srv"
  "srv/terra/GetTopology.srv"
  "srv/terra/FinishMapping.srv"
  "srv/terra/QueryPoi.srv"
  "srv/terra/DeletePoi.srv"
  "srv/terra/SetPose.srv"
  "srv/terra/GetInitPose.srv"
  "srv/terra/GetSLAMState.srv"
  "srv/terra/ActiveRelocation.srv"
  "srv/terra/BindTopologyEdge.srv"
  "srv/terra/DeleteNode.srv"
  "srv/terra/StartExploration.srv"
  "srv/terra/EndExploration.srv"
  "srv/terra/GetExplorationStatus.srv"
  "srv/embodied/GetCurJointsPose.srv"
  "srv/embodied/GetCurTcpTwist.srv"
  "srv/embodied/GetCurTcpPose.srv"
  "srv/TopoRoute.srv"
  "srv/embodied/ArmEnable.srv"
  "srv/embodied/ClearErrorCode.srv"
  "srv/embodied/GetArmEnableStatus.srv"
  "srv/embodied/GetArmStatus.srv"
  "srv/embodied/GetErrorCode.srv"
  "srv/embodied/SetControlMode.srv"
  "srv/embodied/SetCartesianImpedanceDamping.srv"
  "srv/embodied/SetCartesianImpedanceStiffness.srv"
  "srv/embodied/SetJointAdmittanceDamping.srv"
  "srv/embodied/SetJointAdmittanceStiffness.srv"
  "srv/embodied/GetJointAdmittanceDamping.srv"
  "srv/embodied/GetJointAdmittanceStiffness.srv"
  "srv/embodied/SetJointImpedanceDamping.srv"
  "srv/embodied/SetJointImpedanceStiffness.srv"
  "srv/embodied/StopMotion.srv"
  "srv/embodied/PauseMotion.srv"
  "srv/embodied/ResumeMotion.srv"
  "srv/embodied/CancelMotion.srv"
  "srv/embodied/GetCurrentJointPositionArray.srv"
  "srv/embodied/GetCurrentJointVelocitiesArray.srv"
  "srv/embodied/SetMotionMode.srv"
  "srv/embodied/GetMotionMode.srv"
  "srv/embodied/GetControlMode.srv"
  "srv/embodied/SetTcpOffset.srv"
  "srv/embodied/GetTcpOffset.srv"
  "srv/embodied/GetHandGripperInertial.srv"
  "srv/embodied/SetHandGripperInertial.srv"
  "srv/package-deliever/OpenCompartment.srv"
  "srv/package-deliever/CloseCompartment.srv"
  "srv/package-deliever/SlideOutPackage.srv"
  "srv/package-deliever/SetTailGateLocal.srv"
  "srv/package-deliever/SetTopLidLocal.srv"
  "srv/package-deliever/ConnectCompartment.srv"
  "srv/JDR_SLAM/MapDataBase.srv"
  "srv/JDR_SLAM/GetTimestampedPose.srv"
  "srv/JDR_SLAM/SetPosePrior.srv"
  # action
  "action/SetArmLeftPos.action"
  "action/SetArmRightPos.action"
  "action/SetArmLeftPosII.action"
  "action/SetArmRightPosII.action"
  "action/SetArmLeftCycle.action"
  "action/SetArmRightCycle.action"
  "action/SetArmAction.action"
  "action/SetUpperAction.action"
  "action/SetChassisHeight.action"
  "action/SetWaistPosition.action"
  "action/SetHeadPos.action"
  "action/SetRgbdPos.action"
  "action/MapGenerate.action"
  "action/FaceRegister.action"
  "action/Exploration.action"
  "action/Follow.action"
  "action/DockRobot.action"
  "action/AutoCharge.action"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs otel_interface
)

# Interrupted builds can leave an empty python package directory where
# ament_cmake_python expects a symlink, causing:
#   "existing path cannot be removed: Is a directory"
set(_jdbot_py_pkg "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}")
if(EXISTS "${_jdbot_py_pkg}" AND IS_DIRECTORY "${_jdbot_py_pkg}")
  file(GLOB _jdbot_py_so "${_jdbot_py_pkg}/*.so")
  if(NOT _jdbot_py_so)
    message(STATUS "Removing stale incomplete ${PROJECT_NAME} python directory")
    file(REMOVE_RECURSE "${_jdbot_py_pkg}")
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
