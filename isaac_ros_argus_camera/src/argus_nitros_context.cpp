// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "isaac_ros_argus_camera/argus_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace argus
{

constexpr char ARGUS_NITROS_CONTEXT_YAML[] =
  "config/argus_nitros_context_graph.yaml";

const std::vector<std::pair<std::string, std::string>> ARGUS_EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
  {"gxf_isaac_argus", "gxf/lib/libgxf_isaac_argus.so"}
};

std::unique_ptr<nvidia::isaac_ros::nitros::NitrosContext> g_argus_nitros_context;
std::mutex g_argus_nitros_context_mutex;
bool g_argus_nitros_context_initialized = false;
bool g_argus_nitros_context_destroyed = true;

nvidia::isaac_ros::nitros::NitrosContext & GetArgusNitrosContext()
{
  // Mutex: g_argus_nitros_context_mutex
  const std::lock_guard<std::mutex> lock(g_argus_nitros_context_mutex);
  gxf_result_t code;
  if (g_argus_nitros_context_initialized == false) {
    g_argus_nitros_context = std::make_unique<nvidia::isaac_ros::nitros::NitrosContext>();
    const std::string argus_package_share_directory =
      ament_index_cpp::get_package_share_directory("isaac_ros_argus_camera");

    // Load extensions
    for (const auto & extension_pair : ARGUS_EXTENSIONS) {
      const std::string package_directory =
        ament_index_cpp::get_package_share_directory(extension_pair.first);
      code = g_argus_nitros_context->loadExtension(package_directory, extension_pair.second);
      if (code != GXF_SUCCESS) {
        std::stringstream error_msg;
        error_msg << "loadExtensions Error: " << GxfResultStr(code);
        RCLCPP_ERROR(rclcpp::get_logger("ArgusNitrosContext"), error_msg.str().c_str());
        throw std::runtime_error(error_msg.str().c_str());
      }
    }

    // Load application
    code = g_argus_nitros_context->loadApplication(
      argus_package_share_directory + "/" + ARGUS_NITROS_CONTEXT_YAML);
    if (code != GXF_SUCCESS) {
      std::stringstream error_msg;
      error_msg << "loadApplication Error: " << GxfResultStr(code);
      RCLCPP_ERROR(rclcpp::get_logger("ArgusNitrosContext"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    // Run graph
    code = g_argus_nitros_context->runGraphAsync();
    if (code != GXF_SUCCESS) {
      std::stringstream error_msg;
      error_msg << "runGraphAsync Error: " << GxfResultStr(code);
      RCLCPP_ERROR(rclcpp::get_logger("ArgusNitrosContext"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }


    g_argus_nitros_context_initialized = true;
    g_argus_nitros_context_destroyed = false;
  }
  return *g_argus_nitros_context;
  // End Mutex: g_argus_nitros_context_mutex
}

void DestroyArgusNitrosContext()
{
  // Mutex: g_argus_nitros_context_mutex
  const std::lock_guard<std::mutex> lock(g_argus_nitros_context_mutex);
  if (!g_argus_nitros_context_destroyed) {
    g_argus_nitros_context->destroy();
    g_argus_nitros_context.reset();
    g_argus_nitros_context_destroyed = true;
    g_argus_nitros_context_initialized = false;
  }
  // End Mutex: g_argus_nitros_context_mutex
}

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia
