/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_MONO_NODE_HPP_
#define ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_MONO_NODE_HPP_

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

class ArgusMonoNode : public ArgusCameraNode
{
public:
  explicit ArgusMonoNode(const rclcpp::NodeOptions & options);
  ~ArgusMonoNode();

private:
  std::string optical_frame_name_;

  std::string camera_info_url_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
};

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_ARGUS_CAMERA__ARGUS_CAMERA_MONO_NODE_HPP_
