/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_ARGUS_CAMERA_STEREO__ARGUS_CAMERA_STEREO_NODE_HPP_
#define ISAAC_ROS_ARGUS_CAMERA_STEREO__ARGUS_CAMERA_STEREO_NODE_HPP_

#include <cmath>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#define POLYNOMIAL_DISTORTION_COEFFICIENT_COUNT 8
#define POLYNOMIAL_RADIAL_DISTORTION_COEFFICIENT_COUNT 6

#define MAX_MODULE_STRING 32
#define MAX_CAM_DEVICE 6

namespace isaac_ros
{
namespace argus
{

class ArgusStereoNode : public rclcpp::Node
{
public:
  explicit ArgusStereoNode(const rclcpp::NodeOptions &);

  virtual ~ArgusStereoNode();

private:
  // Publishers used for inter process comm
  image_transport::CameraPublisher publishers_[MAX_CAM_DEVICE];

  sensor_msgs::msg::CameraInfo::SharedPtr camerainfo_[MAX_CAM_DEVICE];
  sensor_msgs::msg::Image::SharedPtr images_[MAX_CAM_DEVICE];

  std::string encoding_;
  uint32_t width_;
  uint32_t height_;
};

}  // namespace argus
}  // namespace isaac_ros

#endif  // ISAAC_ROS_ARGUS_CAMERA_STEREO__ARGUS_CAMERA_STEREO_NODE_HPP_
