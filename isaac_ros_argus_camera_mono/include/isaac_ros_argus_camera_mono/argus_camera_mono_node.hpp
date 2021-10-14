/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_ARGUS_CAMERA_MONO__ARGUS_CAMERA_MONO_NODE_HPP_
#define ISAAC_ROS_ARGUS_CAMERA_MONO__ARGUS_CAMERA_MONO_NODE_HPP_

#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#define POLYNOMIAL_DISTORTION_COEFFICIENT_COUNT 8
#define POLYNOMIAL_RADIAL_DISTORTION_COEFFICIENT_COUNT 6

namespace isaac_ros
{
namespace argus
{

class MonocularNode : public rclcpp::Node
{
public:
  explicit MonocularNode(const rclcpp::NodeOptions &);

  virtual ~MonocularNode();

private:
  // Publisher used for intra process comm
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_pub_;

  // Publisher used for inter process comm
  image_transport::CameraPublisher publisher_;

  sensor_msgs::msg::CameraInfo::SharedPtr camerainfo_;
  sensor_msgs::msg::Image::SharedPtr image_;

  uint32_t width_;
  uint32_t height_;
};

}  // namespace argus
}  // namespace isaac_ros

#endif  // ISAAC_ROS_ARGUS_CAMERA_MONO__ARGUS_CAMERA_MONO_NODE_HPP_
