// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_argus_camera/argus_camera_mono_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace argus
{
namespace
{

constexpr char OUTPUT_COMPONENT_KEY_LEFT_IMAGE[] = "sink_left_image/sink";
constexpr char OUTPUT_TOPIC_NAME_LEFT_IMAGE[] = "left/image_raw";

constexpr char OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO[] = "sink_left_camerainfo/sink";
constexpr char OUTPUT_TOPIC_NAME_LEFT_CAMERAINFO[] = "left/camerainfo";

constexpr char OUTPUT_DEFAULT_IMAGE_FORMAT[] = "nitros_image_rgb8";
constexpr char OUTPUT_DEFAULT_CAMERAINFO_FORMAT[] = "nitros_camera_info";

constexpr char APP_YAML_FILENAME[] = "config/argus_camera_mono_node.yaml";

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {OUTPUT_COMPONENT_KEY_LEFT_IMAGE,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = OUTPUT_DEFAULT_IMAGE_FORMAT,
      .topic_name = OUTPUT_TOPIC_NAME_LEFT_IMAGE
    }
  },
  {OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = OUTPUT_DEFAULT_CAMERAINFO_FORMAT,
      .topic_name = OUTPUT_TOPIC_NAME_LEFT_CAMERAINFO
    }
  }
};
#pragma GCC diagnostic pop

constexpr char PACKAGE_NAME[] = "isaac_ros_argus_camera";
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_argus_camera_mono",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
  {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_gxf_helpers.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_sight.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_atlas.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so"},
  {"isaac_ros_gxf", "gxf/lib/multimedia/libgxf_multimedia.so"},
  {"isaac_ros_image_proc", "gxf/lib/image_proc/libgxf_tensorops.so"},
  {"isaac_ros_image_proc", "gxf/lib/image_proc/libgxf_rectify_params_generator.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_timestamp_correlator.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_argus.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_message_compositor.so"}
};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule.yaml"
};
}  // namespace

ArgusMonoNode::ArgusMonoNode(const rclcpp::NodeOptions & options)
: ArgusCameraNode(
    options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME)
{
  camera_id_ = declare_parameter<int>("camera_id", 0);
  module_id_ = declare_parameter<int>("module_id", 0);
  mode_ = declare_parameter<int>("mode", 0);
  fsync_type_ = declare_parameter<int>("fsync_type", 1);
  camera_type_ = declare_parameter<int>("camera_type", 0);
  camera_link_frame_name_ = declare_parameter<std::string>("camera_link_frame_name", "camera");
  optical_frame_name_ = declare_parameter<std::string>("optical_frame_name", "left_cam");
  camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");

  // Load camera info from a file if provided
  if (!camera_info_url_.empty()) {
    camera_info_ = loadCameraInfoFromFile(camera_info_url_);
    RCLCPP_INFO(
      get_logger(), "[ArgusMonoNode] Loaded camera info from \"%s\"", camera_info_url_.c_str());
  }

  // Adding callback for left image
  config_map_[OUTPUT_COMPONENT_KEY_LEFT_IMAGE].callback =
    std::bind(
    &ArgusCameraNode::ArgusImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, optical_frame_name_);

  // Adding callback for left camerainfo
  config_map_[OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO].callback =
    std::bind(
    &ArgusCameraNode::ArgusCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_, optical_frame_name_,
    camera_info_);

  startNitrosNode();
}

ArgusMonoNode::~ArgusMonoNode() = default;

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::argus::ArgusMonoNode)
