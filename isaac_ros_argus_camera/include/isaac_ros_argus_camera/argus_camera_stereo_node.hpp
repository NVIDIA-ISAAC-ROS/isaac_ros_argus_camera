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

#ifndef ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_STEREO_NODE_HPP_
#define ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_STEREO_NODE_HPP_

#include <string>

#include "isaac_ros_argus_camera/argus_camera_node.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace argus
{

class ArgusStereoNode : public ArgusCameraNode
{
public:
  explicit ArgusStereoNode(const rclcpp::NodeOptions & options);
  ~ArgusStereoNode();

  // The callback to be implemented by users for any required initialization
  void postLoadGraphCallback() override;

private:
  std::string left_camera_frame_name_;
  std::string right_camera_frame_name_;

  std::string left_camera_info_url_;
  std::string right_camera_info_url_;

  sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info_;

  bool wide_fov_;
};

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_STEREO_NODE_HPP_
