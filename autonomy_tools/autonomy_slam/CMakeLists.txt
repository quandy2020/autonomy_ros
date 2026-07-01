cmake_minimum_required(VERSION 3.5)
project(autonomy_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Optional tooling
option(USE_SANITIZER "Enable AddressSanitizer" OFF)
option(BUILD_WITH_MARCH_NATIVE "Enable -march=native" OFF)

add_compile_options(-Wall -Wextra)
if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-mtune=native -march=native)
else()
  add_compile_options(-mtune=native)
endif()
if(USE_SANITIZER)
  add_compile_options(-fno-omit-frame-pointer -fsanitize=address)
  add_link_options(-fno-omit-frame-pointer -fsanitize=address)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# autonomy::autonomy needs TinyXML2::TinyXML2 before AutonomyTargets.cmake is loaded.
if(NOT TARGET TinyXML2::TinyXML2)
  foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
    if(EXISTS "${_prefix}/share/autonomy/cmake/modules/FindTinyXML2.cmake")
      include("${_prefix}/share/autonomy/cmake/modules/FindTinyXML2.cmake")
      break()
    endif()
  endforeach()
endif()
if(NOT TARGET TinyXML2::TinyXML2)
  foreach(_mod_dir IN ITEMS
      "${CMAKE_CURRENT_SOURCE_DIR}/../../autonomy_ros/cmake/modules"
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    if(EXISTS "${_mod_dir}/FindTinyXML2.cmake")
      include("${_mod_dir}/FindTinyXML2.cmake")
      break()
    endif()
  endforeach()
endif()
if(NOT TARGET TinyXML2::TinyXML2)
  message(FATAL_ERROR "autonomy_slam: failed to import TinyXML2::TinyXML2")
endif()

# All targets linking autonomy::autonomy must use /usr/local OpenCV 5.x (same ABI as atlas).
set(AUTONOMY_SLAM_ATLAS_OPENCV_INCLUDES "/usr/include/opencv4")
if(EXISTS "/usr/local/include/opencv5")
  set(AUTONOMY_SLAM_ATLAS_OPENCV_INCLUDES "/usr/local/include/opencv5")
endif()
set(AUTONOMY_SLAM_ATLAS_OPENCV_LIBS "")
foreach(_lib IN ITEMS
    /usr/local/lib/libopencv_core.so
    /usr/local/lib/libopencv_imgproc.so
    /usr/local/lib/libopencv_imgcodecs.so
    /usr/local/lib/libopencv_videoio.so
    /usr/local/lib/libopencv_calib3d.so)
  if(EXISTS "${_lib}")
    list(APPEND AUTONOMY_SLAM_ATLAS_OPENCV_LIBS "${_lib}")
  endif()
endforeach()
if(NOT AUTONOMY_SLAM_ATLAS_OPENCV_LIBS)
  message(FATAL_ERROR "autonomy_slam: /usr/local OpenCV 5.x not found (required by autonomy::autonomy)")
endif()

find_package(autonomy REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(AUTONOMY_SLAM_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(AUTONOMY_SLAM_COMMON_SOURCES
  ${AUTONOMY_SLAM_SRC_DIR}/autonomy_slam.cpp
  ${AUTONOMY_SLAM_SRC_DIR}/slam_cli.cpp
  ${AUTONOMY_SLAM_SRC_DIR}/stereo_rectifier.cpp)

set(AUTONOMY_SLAM_ROS_DEPS
  rclcpp cv_bridge image_transport message_filters
  geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2 tf2_eigen tf2_geometry_msgs tf2_msgs tf2_ros)

function(autonomy_slam_configure_target target)
  target_include_directories(${target} BEFORE PRIVATE ${AUTONOMY_SLAM_ATLAS_OPENCV_INCLUDES})
  target_include_directories(${target} PRIVATE ${AUTONOMY_SLAM_SRC_DIR})
  target_link_libraries(${target} PRIVATE yaml-cpp autonomy::autonomy ${AUTONOMY_SLAM_ATLAS_OPENCV_LIBS})
  ament_target_dependencies(${target} PUBLIC ${AUTONOMY_SLAM_ROS_DEPS})
endfunction()

function(autonomy_slam_configure_executable target)
  autonomy_slam_configure_target(${target})
  ament_target_dependencies(${target} PUBLIC rosbag2_cpp)
  # fbow/g2o live in /usr/local/lib; transitive g2o libs need this at runtime.
  set_target_properties(${target} PROPERTIES
    INSTALL_RPATH "/usr/local/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

function(autonomy_slam_configure_video_executable target)
  target_include_directories(${target} BEFORE PRIVATE ${AUTONOMY_SLAM_ATLAS_OPENCV_INCLUDES})
  target_include_directories(${target} PRIVATE ${AUTONOMY_SLAM_SRC_DIR})
  target_link_libraries(${target} PRIVATE yaml-cpp autonomy::autonomy ${AUTONOMY_SLAM_ATLAS_OPENCV_LIBS})
  set_target_properties(${target} PROPERTIES
    INSTALL_RPATH "/usr/local/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

# Composable node library
add_library(autonomy_slam_system SHARED
  ${AUTONOMY_SLAM_SRC_DIR}/system.cpp
  ${AUTONOMY_SLAM_COMMON_SOURCES})
autonomy_slam_configure_target(autonomy_slam_system)
ament_target_dependencies(autonomy_slam_system PUBLIC rclcpp_components)
rclcpp_components_register_node(
  autonomy_slam_system PLUGIN "autonomy_slam::System" EXECUTABLE system)

# CLI executables
foreach(_exe IN ITEMS run_slam run_slam_offline)
  add_executable(${_exe}
    ${AUTONOMY_SLAM_SRC_DIR}/${_exe}.cpp
    ${AUTONOMY_SLAM_SRC_DIR}/slam_cli_ros.cpp
    ${AUTONOMY_SLAM_COMMON_SOURCES})
  autonomy_slam_configure_executable(${_exe})
endforeach()

add_executable(run_video_slam
  ${AUTONOMY_SLAM_SRC_DIR}/run_video_slam.cpp
  ${AUTONOMY_SLAM_SRC_DIR}/slam_cli.cpp)
autonomy_slam_configure_video_executable(run_video_slam)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(NOT WIN32)
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/usr_local_lib.sh"
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS autonomy_slam_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config launch rviz vocab
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
install(PROGRAMS scripts/video_publisher.py scripts/aist_tutorial.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
