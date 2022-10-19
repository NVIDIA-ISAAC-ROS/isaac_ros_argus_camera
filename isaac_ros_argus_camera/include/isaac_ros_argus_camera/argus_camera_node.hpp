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

#ifndef ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_NODE_HPP_
#define ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "tf2_ros/transform_broadcaster.h"

#include "isaac_ros_nitros/nitros_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace argus
{

class ArgusCameraNode : public nitros::NitrosNode
{
public:
  explicit ArgusCameraNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const nitros::NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & preset_extension_spec_names,
    const std::vector<std::string> & extension_spec_filenames);

  ~ArgusCameraNode();

  void preLoadGraphCallback() override;
  void postLoadGraphCallback() override;

  // Callback to publish camera extrinsics to ROS TF tree
  void ArgusCameraInfoCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string parent_frame, const std::string child_frame,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  // Callback to set frame_id
  void ArgusImageCallback(
    const gxf_context_t context, nitros::NitrosTypeBase & msg,
    const std::string frame_name);

  sensor_msgs::msg::CameraInfo::SharedPtr loadCameraInfoFromFile(
    const std::string camera_info_url);

protected:
  int camera_id_;
  int module_id_;
  int mode_;
  int camera_type_;
  std::string camera_link_frame_name_;

private:
  // Publisher for tf2.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
};

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_NODE_HPP_
