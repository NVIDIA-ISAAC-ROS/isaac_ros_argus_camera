# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_argus_camera LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
ament_auto_find_build_dependencies()

ament_auto_add_library(argus_nitros_context SHARED src/argus_nitros_context.cpp)

ament_auto_add_library(base_node SHARED src/argus_camera_node.cpp)
target_link_libraries(base_node Eigen3::Eigen)
set_target_properties(base_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

ament_auto_add_library(mono_node SHARED src/argus_camera_mono_node.cpp)
target_link_libraries(mono_node Eigen3::Eigen)
rclcpp_components_register_nodes(mono_node "nvidia::isaac_ros::argus::ArgusMonoNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::argus::ArgusMonoNode;$<TARGET_FILE:mono_node>\n")
set_target_properties(mono_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

ament_auto_add_library(stereo_node SHARED src/argus_camera_stereo_node.cpp)
target_link_libraries(stereo_node Eigen3::Eigen)
rclcpp_components_register_nodes(stereo_node "nvidia::isaac_ros::argus::ArgusStereoNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::argus::ArgusStereoNode;$<TARGET_FILE:stereo_node>\n")
set_target_properties(stereo_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  # Only add camera tests if on Jetson platform
  if(${ARCHITECTURE} STREQUAL "aarch64")
  add_launch_test(test/isaac_ros_argus_camera_bench_test.py)
  add_launch_test(test/isaac_ros_argus_camera_mono_test.py)
  add_launch_test(test/isaac_ros_argus_camera_stereo_test.py)
  add_launch_test(test/isaac_ros_argus_camera_stereo_oss_rectify_compare.py)
  add_launch_test(test/isaac_ros_argus_camera_stereo_wide_fov_test.py)
  add_launch_test(test/isaac_ros_argus_camera_override_test.py)
  add_launch_test(test/isaac_ros_argus_camera_mono_override_test.py)
  endif()  # aarch64
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch test)
