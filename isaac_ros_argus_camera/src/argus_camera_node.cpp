// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gxf/multimedia/camera.hpp"
#include "gxf/std/timestamp.hpp"
#include "isaac_ros_argus_camera/argus_camera_node.hpp"
#include "isaac_ros_nitros_camera_info_type/nitros_camera_info.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace argus
{
namespace
{

using DistortionType = nvidia::gxf::DistortionType;
const std::unordered_map<std::string, DistortionType> g_ros_to_gxf_distortion_model({
          {"pinhole", DistortionType::Perspective},
          {"plumb_bob", DistortionType::Polynomial},
          {"rational_polynomial", DistortionType::Polynomial},
          {"equidistant", DistortionType::FisheyeEquidistant}
        });

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
  {"isaac_ros_gxf", "gxf/lib/libgxf_argus.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_message_compositor.so"}
};
constexpr char PACKAGE_NAME[] = "isaac_ros_argus_camera";
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule.yaml"
};
}  // namespace

ArgusCameraNode::ArgusCameraNode(
  const rclcpp::NodeOptions & options,
  const std::string & app_yaml_filename,
  const nitros::NitrosPublisherSubscriberConfigMap & config_map,
  const std::vector<std::string> & preset_extension_spec_names,
  const std::vector<std::string> & extension_spec_filenames)
: nitros::NitrosNode(options,
    app_yaml_filename,
    config_map,
    preset_extension_spec_names,
    extension_spec_filenames,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  camera_link_frame_name_("camera"),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosCameraInfo>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImage>();
}

void ArgusCameraNode::ArgusImageCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg, const std::string frame_name)
{
  (void)context;
  msg.frame_id = frame_name;
}

void ArgusCameraNode::ArgusCameraInfoCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg,
  const std::string parent_frame, const std::string child_frame,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);

  // Fill in CameraModel if camera info is provided
  if (camera_info != nullptr) {
    RCLCPP_DEBUG(
      get_logger(), "[ArgusCameraNode] Overriding CameraModel with vaules loaded from URL");

    auto gxf_camera_model = msg_entity->get<nvidia::gxf::CameraModel>();
    if (!gxf_camera_model) {
      std::stringstream error_msg;
      error_msg <<
        "[ArgusCameraNode] Failed to get the existing CameraModel object: " <<
        GxfResultStr(gxf_camera_model.error());
      RCLCPP_ERROR(
        get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    gxf_camera_model.value()->dimensions = {camera_info->width, camera_info->height};
    gxf_camera_model.value()->focal_length = {
      static_cast<float>(camera_info->k[0]), static_cast<float>(camera_info->k[4])};
    gxf_camera_model.value()->principal_point = {
      static_cast<float>(camera_info->k[2]), static_cast<float>(camera_info->k[5])};

    const auto distortion = g_ros_to_gxf_distortion_model.find(camera_info->distortion_model);
    if (distortion == std::end(g_ros_to_gxf_distortion_model)) {
      std::stringstream error_msg;
      error_msg <<
        "[ArgusCameraNode] Unsupported distortion model from ROS \"" <<
        camera_info->distortion_model.c_str() << "\"";
      RCLCPP_ERROR(
        get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    } else {
      gxf_camera_model.value()->distortion_type = distortion->second;
    }

    if (gxf_camera_model.value()->distortion_type == DistortionType::Polynomial) {
      // prevents distortion parameters array access if its empty
      // simulators may send empty distortion parameter array since images are already rectified
      if (!camera_info->d.empty()) {
        // distortion parameters in GXF: k1, k2, k3, k4, k5, k6, p1, p2
        // distortion parameters in ROS message: k1, k2, p1, p2, k3 ...
        gxf_camera_model.value()->distortion_coefficients[0] = camera_info->d[0];
        gxf_camera_model.value()->distortion_coefficients[1] = camera_info->d[1];

        for (uint16_t index = 2; index < camera_info->d.size() - 2; index++) {
          gxf_camera_model.value()->distortion_coefficients[index] = camera_info->d[index + 2];
        }
        gxf_camera_model.value()->distortion_coefficients[6] = camera_info->d[2];
        gxf_camera_model.value()->distortion_coefficients[7] = camera_info->d[3];
      }
    } else {
      std::copy(
        std::begin(camera_info->d), std::end(camera_info->d),
        std::begin(gxf_camera_model.value()->distortion_coefficients));
    }

    // Set extrinsic information into message
    auto gxf_pose_3d = msg_entity->get<nvidia::gxf::Pose3D>();
    if (!gxf_pose_3d) {
      std::stringstream error_msg;
      error_msg << "[ArgusCameraNode] Failed to get Pose3D object from message entity: " <<
        GxfResultStr(gxf_pose_3d.error());
      RCLCPP_ERROR(
        get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    std::copy(
      std::begin(camera_info->r), std::end(camera_info->r),
      std::begin(gxf_pose_3d.value()->rotation));
    if (camera_info->p[0] == 0.0f) {
      gxf_pose_3d.value()->translation = {
        static_cast<float>(camera_info->p[3]),
        static_cast<float>(camera_info->p[7]),
        static_cast<float>(camera_info->p[11])};
    } else {
      gxf_pose_3d.value()->translation = {
        static_cast<float>(camera_info->p[3]) / static_cast<float>(camera_info->p[0]),
        static_cast<float>(camera_info->p[7]),
        static_cast<float>(camera_info->p[11])};
    }
  }

  // Populate timestamp information
  auto gxf_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  if (!gxf_timestamp) {    // Fallback to label 'timestamp'
    gxf_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  }
  if (gxf_timestamp) {
    transform_stamped.header.stamp.sec = static_cast<int32_t>(
      gxf_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    transform_stamped.header.stamp.nanosec = static_cast<uint32_t>(
      gxf_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[ArgusCameraNode] Failed to get timestamp");
  }

  // Extract camera extrinsics
  auto gxf_pose_3d = msg_entity->get<nvidia::gxf::Pose3D>();
  if (!gxf_pose_3d) {
    std::stringstream error_msg;
    error_msg << "[ArgusCameraNode] Failed to get Pose3D object from message entity: " <<
      GxfResultStr(gxf_pose_3d.error());
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  msg.frame_id = child_frame;

  transform_stamped.header.frame_id = parent_frame;
  transform_stamped.child_frame_id = child_frame;
  transform_stamped.transform.translation.x = gxf_pose_3d.value()->translation[0];
  transform_stamped.transform.translation.y = gxf_pose_3d.value()->translation[1];
  transform_stamped.transform.translation.z = gxf_pose_3d.value()->translation[2];
  // tf2::Matrix3x3 is row major
  tf2::Matrix3x3 rot_mat = {gxf_pose_3d.value()->rotation[0],
    gxf_pose_3d.value()->rotation[3],
    gxf_pose_3d.value()->rotation[6],
    gxf_pose_3d.value()->rotation[1],
    gxf_pose_3d.value()->rotation[4],
    gxf_pose_3d.value()->rotation[7],
    gxf_pose_3d.value()->rotation[2],
    gxf_pose_3d.value()->rotation[5],
    gxf_pose_3d.value()->rotation[8]};
  tf2::Quaternion quaternion;
  rot_mat.getRotation(quaternion);
  transform_stamped.transform.rotation.x = quaternion.x();
  transform_stamped.transform.rotation.y = quaternion.y();
  transform_stamped.transform.rotation.z = quaternion.z();
  transform_stamped.transform.rotation.w = quaternion.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void ArgusCameraNode::preLoadGraphCallback() {}

void ArgusCameraNode::postLoadGraphCallback()
{
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "camera_id", camera_id_);
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "module_id", module_id_);
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "mode", mode_);
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "camera_type", camera_type_);
}

sensor_msgs::msg::CameraInfo::SharedPtr ArgusCameraNode::loadCameraInfoFromFile(
  const std::string camera_info_url)
{
  // Load camera info with calibration data from the URL
  std::string camera_name = camera_info_url.substr(camera_info_url.find_last_of("/\\") + 1);
  camera_name = camera_name.substr(0, camera_name.find_last_of("."));

  camera_info_manager::CameraInfoManager cinfo(this, camera_name, camera_info_url);
  if (cinfo.validateURL(camera_info_url)) {
    if (cinfo.isCalibrated()) {
      return std::make_shared<sensor_msgs::msg::CameraInfo>(
        sensor_msgs::msg::CameraInfo(cinfo.getCameraInfo()));
    } else {
      std::stringstream error_msg;
      error_msg << "[ArgusCameraNode] Camera info " << camera_name << " not calibrated";
      RCLCPP_ERROR(
        get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  } else {
    std::stringstream error_msg;
    error_msg << "[ArgusCameraNode] Unable to validate camera info URL: " << camera_name;
    RCLCPP_ERROR(
      get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
}

ArgusCameraNode::~ArgusCameraNode() = default;

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia
