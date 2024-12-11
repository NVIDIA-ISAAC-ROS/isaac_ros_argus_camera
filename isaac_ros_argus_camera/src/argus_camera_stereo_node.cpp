// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_argus_camera/argus_camera_stereo_node.hpp"

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

constexpr char OUTPUT_COMPONENT_KEY_RIGHT_IMAGE[] = "sink_right_image/sink";
constexpr char OUTPUT_TOPIC_NAME_RIGHT_IMAGE[] = "right/image_raw";

constexpr char OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO[] = "sink_left_camera_info/sink";
constexpr char OUTPUT_TOPIC_NAME_LEFT_CAMERAINFO[] = "left/camera_info";

constexpr char OUTPUT_COMPONENT_KEY_RIGHT_CAMERAINFO[] = "sink_right_camera_info/sink";
constexpr char OUTPUT_TOPIC_NAME_RIGHT_CAMERAINFO[] = "right/camera_info";

constexpr char OUTPUT_DEFAULT_IMAGE_FORMAT[] = "nitros_image_rgb8";
constexpr char OUTPUT_DEFAULT_CAMERAINFO_FORMAT[] = "nitros_camera_info";

constexpr char APP_YAML_FILENAME[] = "config/argus_camera_stereo_node.yaml";

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
  {OUTPUT_COMPONENT_KEY_RIGHT_IMAGE,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = OUTPUT_DEFAULT_IMAGE_FORMAT,
      .topic_name = OUTPUT_TOPIC_NAME_RIGHT_IMAGE
    }
  },
  {OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = OUTPUT_DEFAULT_CAMERAINFO_FORMAT,
      .topic_name = OUTPUT_TOPIC_NAME_LEFT_CAMERAINFO
    }
  },
  {OUTPUT_COMPONENT_KEY_RIGHT_CAMERAINFO,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = OUTPUT_DEFAULT_CAMERAINFO_FORMAT,
      .topic_name = OUTPUT_TOPIC_NAME_RIGHT_CAMERAINFO
    }
  }
};
#pragma GCC diagnostic pop

constexpr char PACKAGE_NAME[] = "isaac_ros_argus_camera";
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {
  "config/isaac_ros_argus_camera_stereo_spec.yaml",
};
const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"gxf_isaac_gxf_helpers", "gxf/lib/libgxf_isaac_gxf_helpers.so"},
  {"gxf_isaac_sight", "gxf/lib/libgxf_isaac_sight.so"},
  {"gxf_isaac_atlas", "gxf/lib/libgxf_isaac_atlas.so"},
  {"gxf_isaac_messages", "gxf/lib/libgxf_isaac_messages.so"},
  {"gxf_isaac_tensorops", "gxf/lib/libgxf_isaac_tensorops.so"},
  {"gxf_isaac_rectify", "gxf/lib/libgxf_isaac_rectify.so"},
  {"gxf_isaac_timestamp_correlator", "gxf/lib/libgxf_isaac_timestamp_correlator.so"},
  {"gxf_isaac_argus", "gxf/lib/libgxf_isaac_argus.so"},
  {"gxf_isaac_message_compositor", "gxf/lib/libgxf_isaac_message_compositor.so"},
  {"gxf_isaac_camera_utils", "gxf/lib/libgxf_isaac_camera_utils.so"}
};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule.yaml"
};
}  // namespace

ArgusStereoNode::ArgusStereoNode(const rclcpp::NodeOptions & options)
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
  module_id_ = declare_parameter<int>("module_id", -1);
  mode_ = declare_parameter<int>("mode", 0);
  fsync_type_ = declare_parameter<int>("fsync_type", 1);
  use_hw_timestamp_ = declare_parameter<bool>("use_hw_timestamp", false);
  camera_link_frame_name_ =
    declare_parameter<std::string>("camera_link_frame_name", "stereo_camera");
  left_camera_frame_name_ = declare_parameter<std::string>(
    "left_camera_frame_name",
    "stereo_camera_left");
  right_camera_frame_name_ =
    declare_parameter<std::string>("right_camera_frame_name", "stereo_camera_right");
  left_camera_info_url_ =
    declare_parameter<std::string>("left_camera_info_url", "");
  right_camera_info_url_ =
    declare_parameter<std::string>("right_camera_info_url", "");
  wide_fov_ = declare_parameter<bool>("wide_fov", false);

  // Load camera info from files if provided
  if (!left_camera_info_url_.empty()) {
    left_camera_info_ = loadCameraInfoFromFile(left_camera_info_url_);
    RCLCPP_INFO(
      get_logger(), "[ArgusStereoNode] Loaded left camera info from \"%s\"",
      left_camera_info_url_.c_str());
  }
  if (!right_camera_info_url_.empty()) {
    right_camera_info_ = loadCameraInfoFromFile(right_camera_info_url_);
    RCLCPP_INFO(
      get_logger(), "[ArgusStereoNode] Loaded right camera info from \"%s\"",
      right_camera_info_url_.c_str());
  }

  // Adding callback for left image
  config_map_[OUTPUT_COMPONENT_KEY_LEFT_IMAGE].callback =
    std::bind(
    &ArgusCameraNode::ArgusStereoImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, left_camera_frame_name_);

  // Adding callback for right image
  config_map_[OUTPUT_COMPONENT_KEY_RIGHT_IMAGE].callback =
    std::bind(
    &ArgusCameraNode::ArgusStereoImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, right_camera_frame_name_);

  // Adding callback for left camera_info
  config_map_[OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO].callback =
    std::bind(
    &ArgusStereoNode::ArgusStereoCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_,
    left_camera_frame_name_, left_camera_info_, right_camera_info_, LEFT);

  // Adding callback for right camera_info
  config_map_[OUTPUT_COMPONENT_KEY_RIGHT_CAMERAINFO].callback =
    std::bind(
    &ArgusStereoNode::ArgusStereoCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_,
    right_camera_frame_name_, left_camera_info_, right_camera_info_, RIGHT);

  startNitrosNode();
}

ArgusStereoNode::~ArgusStereoNode() = default;

void ArgusStereoNode::postLoadGraphCallback()
{
  nvidia::isaac_ros::argus::ArgusCameraNode::postLoadGraphCallback();
  RCLCPP_INFO(get_logger(), "[ArgusStereoNode] postLoadGraphCallback().");

  if (wide_fov_) {
    getNitrosContext().setParameterFloat64(
      "rectify_parameters", "nvidia::isaac::RectifyParamsGenerator", "alpha", 0.7);
    RCLCPP_INFO(
      get_logger(), "[ArgusStereoNode] set alpha in rectify parameter generator to  \"%f\"",
      0.7);
  }
}

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::argus::ArgusStereoNode)
