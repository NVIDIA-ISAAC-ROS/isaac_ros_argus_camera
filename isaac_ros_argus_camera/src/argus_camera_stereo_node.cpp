/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_argus_camera/argus_camera_stereo_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace argus
{
namespace
{

constexpr char OUTPUT_COMPONENT_KEY_LEFT_IMAGE[] = "vault_left_image/vault";
constexpr char OUTPUT_TOPIC_NAME_LEFT_IMAGE[] = "left/image_raw";

constexpr char OUTPUT_COMPONENT_KEY_RIGHT_IMAGE[] = "vault_right_image/vault";
constexpr char OUTPUT_TOPIC_NAME_RIGHT_IMAGE[] = "right/image_raw";

constexpr char OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO[] = "vault_left_camerainfo/vault";
constexpr char OUTPUT_TOPIC_NAME_LEFT_CAMERAINFO[] = "left/camerainfo";

constexpr char OUTPUT_COMPONENT_KEY_RIGHT_CAMERAINFO[] = "vault_right_camerainfo/vault";
constexpr char OUTPUT_TOPIC_NAME_RIGHT_CAMERAINFO[] = "right/camerainfo";

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

const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_argus_camera_stereo",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
}  // namespace

ArgusStereoNode::ArgusStereoNode(const rclcpp::NodeOptions & options)
: ArgusCameraNode(
    options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES)
{
  camera_id_ = declare_parameter<int>("camera_id", 0);
  module_id_ = declare_parameter<int>("module_id", 0);
  mode_ = declare_parameter<int>("mode", 0);
  camera_type_ = declare_parameter<int>("camera_type", 1);
  camera_link_frame_name_ = declare_parameter<std::string>("camera_link_frame_name", "camera");
  left_optical_frame_name_ = declare_parameter<std::string>("left_optical_frame_name", "left_cam");
  right_optical_frame_name_ =
    declare_parameter<std::string>("right_optical_frame_name", "right_cam");
  left_camera_info_url_ =
    declare_parameter<std::string>("left_camera_info_url", "");
  right_camera_info_url_ =
    declare_parameter<std::string>("right_camera_info_url", "");

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
    &ArgusCameraNode::ArgusImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, left_optical_frame_name_);

  // Adding callback for right image
  config_map_[OUTPUT_COMPONENT_KEY_RIGHT_IMAGE].callback =
    std::bind(
    &ArgusCameraNode::ArgusImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, right_optical_frame_name_);

  // Adding callback for left camerainfo
  config_map_[OUTPUT_COMPONENT_KEY_LEFT_CAMERAINFO].callback =
    std::bind(
    &ArgusCameraNode::ArgusCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_,
    left_optical_frame_name_, left_camera_info_);

  // Adding callback for right camerainfo
  config_map_[OUTPUT_COMPONENT_KEY_RIGHT_CAMERAINFO].callback =
    std::bind(
    &ArgusCameraNode::ArgusCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_,
    right_optical_frame_name_, right_camera_info_);

  startNitrosNode();
}

ArgusStereoNode::~ArgusStereoNode() = default;

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::argus::ArgusStereoNode)
