# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.23.2)
project(isaac_ros_argus_camera LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Only build Argus nodes on Jetson platform
if(${ARCHITECTURE} STREQUAL "aarch64")
# aarch64

ament_auto_add_library(base_node SHARED src/argus_camera_node.cpp)
ament_auto_add_library(mono_node SHARED src/argus_camera_mono_node.cpp)
rclcpp_components_register_nodes(mono_node "nvidia::isaac_ros::argus::ArgusMonoNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::argus::ArgusMonoNode;$<TARGET_FILE:mono_node>\n")

ament_auto_add_library(stereo_node SHARED src/argus_camera_stereo_node.cpp)
rclcpp_components_register_nodes(stereo_node "nvidia::isaac_ros::argus::ArgusStereoNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::argus::ArgusStereoNode;$<TARGET_FILE:stereo_node>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_argus_camera_mono_test.py)
  add_launch_test(test/isaac_ros_argus_camera_stereo_test.py)
endif()

endif()  # aarch64

ament_auto_package(INSTALL_TO_SHARE config launch)
