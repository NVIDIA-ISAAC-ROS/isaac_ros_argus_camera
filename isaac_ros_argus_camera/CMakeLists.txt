# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.8)
project(isaac_ros_argus_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

execute_process(
  COMMAND uname -m
  COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Architecture: ${ARCHITECTURE}")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# Only build Argus nodes on Jetson platform
if(${ARCHITECTURE} STREQUAL "aarch64")
  # Find VPI dependency
  find_package(vpi REQUIRED)
  # Base class
  ament_auto_add_library(base_node SHARED src/argus_camera_node.cpp)
  # Mono camera node
  ament_auto_add_library(mono_node SHARED src/argus_camera_mono_node.cpp)
  # Stereo camera node
  ament_auto_add_library(stereo_node SHARED src/argus_camera_stereo_node.cpp)

  target_include_directories(
    mono_node
    PRIVATE /usr/lib/aarch64-linux-gnu/tegra
            /usr/src/jetson_multimedia_api/argus/include
            /usr/src/jetson_multimedia_api/argus/samples/utils
            /usr/src/jetson_multimedia_api/include
            include)

  target_include_directories(
    stereo_node
    PRIVATE /usr/lib/aarch64-linux-gnu/tegra
            /usr/src/jetson_multimedia_api/argus/include
            /usr/src/jetson_multimedia_api/argus/samples/utils
            /usr/src/jetson_multimedia_api/include
            include)

  target_compile_definitions(mono_node PRIVATE "COMPOSITION_BUILDING_DLL")

  target_compile_definitions(stereo_node PRIVATE "COMPOSITION_BUILDING_DLL")

  target_link_libraries(
    mono_node
    /usr/lib/aarch64-linux-gnu/tegra/libnvargus.so
    /usr/lib/aarch64-linux-gnu/tegra/libnvargus_socketclient.so
    /usr/lib/aarch64-linux-gnu/tegra/libnvargus_socketserver.so
    /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
    vpi)

  target_link_libraries(
    stereo_node
    /usr/lib/aarch64-linux-gnu/tegra/libnvargus.so
    /usr/lib/aarch64-linux-gnu/tegra/libnvargus_socketclient.so
    /usr/lib/aarch64-linux-gnu/tegra/libnvargus_socketserver.so
    /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
    vpi)

  rclcpp_components_register_nodes(mono_node "nvidia::isaac_ros::argus::ArgusMonoNode")
  set(node_plugins
      "${node_plugins}nvidia::isaac_ros::argus::ArgusMonoNode;$<TARGET_FILE:mono_node>\n"
  )

  rclcpp_components_register_nodes(stereo_node "nvidia::isaac_ros::argus::ArgusStereoNode")
  set(node_plugins
      "${node_plugins}nvidia::isaac_ros::argus::ArgusStereoNode;$<TARGET_FILE:stereo_node>\n"
  )

  # Install extensions directory
  install(
    DIRECTORY gxf/lib/gxf_jetpack50/
    DESTINATION share/${PROJECT_NAME}/gxf)
  # Install config directory
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
  install(
    TARGETS mono_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  install(
    TARGETS stereo_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  if(${ARCHITECTURE} STREQUAL "aarch64")
    add_launch_test(test/isaac_ros_argus_camera_mono_test.py)
    add_launch_test(test/isaac_ros_argus_camera_stereo_test.py)
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
