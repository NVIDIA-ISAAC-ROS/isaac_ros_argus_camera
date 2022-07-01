/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
