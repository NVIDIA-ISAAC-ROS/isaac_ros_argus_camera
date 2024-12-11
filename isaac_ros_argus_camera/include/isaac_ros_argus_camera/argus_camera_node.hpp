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

#ifndef ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_NODE_HPP_
#define ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_NODE_HPP_

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

#include "gxf/multimedia/camera.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace argus
{

enum StereoCameraSide { LEFT, RIGHT };

class ArgusCameraNode : public nitros::NitrosNode
{
public:
  explicit ArgusCameraNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const nitros::NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & preset_extension_spec_names,
    const std::vector<std::string> & extension_spec_filenames,
    const std::vector<std::string> & generator_rule_filenames,
    const std::vector<std::pair<std::string, std::string>> & extensions,
    const std::string & package_name);

  ~ArgusCameraNode();

  void preLoadGraphCallback() override;
  void postLoadGraphCallback() override;

  // Callback to publish camera extrinsics to ROS TF tree
  void ArgusCameraInfoCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string parent_frame, const std::string child_frame,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  // Callback to publish camera extrinsics to ROS TF tree for stereo cameras
  // which may or may not include an IMU
  void ArgusStereoCameraInfoCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string rig_frame, const std::string camera_frame,
    const sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info,
    const sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info,
    const StereoCameraSide side
  );

  // Callback to set frame_id
  void ArgusImageCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string frame_name);

  // Callback to set frame_id in the case of a stereo camera
  void ArgusStereoImageCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string frame_name);

protected:
  int camera_id_;
  int module_id_;
  int mode_;
  int fsync_type_;
  bool use_hw_timestamp_;
  std::string camera_link_frame_name_;
  // Publisher for tf2.
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_{nullptr};
  // set of child frames that have been published
  std::set<std::string> tf_was_published_;

  // Transform that converts from camera body frame to camera optical frame
  // https://www.ros.org/reps/rep-0103.html
  // Camera Link   ->  Optical
  // x             ->  z
  // y             ->  -x
  // z             ->  -y
  tf2::Transform cam_link_pose_optical_{tf2::Matrix3x3{0, 0, 1, -1, 0, 0, 0, -1, 0},
    tf2::Vector3{0, 0, 0}};

  tf2::Transform right_camera_pose_rig_override_;
  bool right_camera_override_tf_set_{false};

  builtin_interfaces::msg::Time timestampFromGxfMessage(
    const nvidia::gxf::Expected<nvidia::gxf::Entity> & gxf_msg_entity);

  tf2::Transform rosTransformFromGxfTransform(
    const nvidia::gxf::Handle<nvidia::gxf::Pose3D> & gxf_pose);

  void fillTransformMsgFromTransform(
    geometry_msgs::msg::TransformStamped & transform_msg,
    const tf2::Transform & transform);

  sensor_msgs::msg::CameraInfo::SharedPtr loadCameraInfoFromFile(
    const std::string camera_info_url);

  void updateMessageFromCameraInfo(
    nvidia::gxf::Expected<nvidia::gxf::Entity> & msg_entity,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  tf2::Transform computeLeftOpticalPoseRightOptical(
    const sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info,
    const sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info);
};

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_NODE_HPP_
