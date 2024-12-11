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

#include <Eigen/Dense>

#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gxf/std/timestamp.hpp"
#include "isaac_ros_argus_camera/argus_camera_node.hpp"
#include "isaac_ros_argus_camera/argus_nitros_context.hpp"
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
          {"plumb_bob", DistortionType::Perspective},
          {"plumb_bob", DistortionType::Brown},
          {"rational_polynomial", DistortionType::Polynomial},
          {"equidistant", DistortionType::FisheyeEquidistant}
        });

constexpr char GXF_EXTRINSICS_NAME[] = "extrinsics";
constexpr char IMU_EXTRINSICS_NAME[] = "imu_extrinsics";
}  // namespace

ArgusCameraNode::ArgusCameraNode(
  const rclcpp::NodeOptions & options,
  const std::string & app_yaml_filename,
  const nitros::NitrosPublisherSubscriberConfigMap & config_map,
  const std::vector<std::string> & preset_extension_spec_names,
  const std::vector<std::string> & extension_spec_filenames,
  const std::vector<std::string> & generator_rule_filenames,
  const std::vector<std::pair<std::string, std::string>> & extensions,
  const std::string & package_name)
: nitros::NitrosNode(options,
    app_yaml_filename,
    config_map,
    preset_extension_spec_names,
    extension_spec_filenames,
    generator_rule_filenames,
    extensions,
    package_name),
  camera_link_frame_name_("camera"),
  tf_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this))
{
  // Start the argus NITROS context gxf graph
  GetArgusNitrosContext();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosCameraInfo>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImage>();
}

void ArgusCameraNode::ArgusImageCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg, const std::string frame_name)
{
  (void)context;
  msg.frame_id = frame_name;
}

void ArgusCameraNode::ArgusStereoImageCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg, const std::string frame_name)
{
  (void)context;
  const std::string optical_frame_name = frame_name + "_optical";
  msg.frame_id = optical_frame_name;
}

builtin_interfaces::msg::Time ArgusCameraNode::timestampFromGxfMessage(
  const nvidia::gxf::Expected<nvidia::gxf::Entity> & msg_entity)
{
  builtin_interfaces::msg::Time extracted_time;
  auto gxf_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  if (!gxf_timestamp) {       // Fallback to label 'timestamp'
    gxf_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  }
  if (gxf_timestamp) {
    extracted_time.sec = static_cast<int32_t>(
      gxf_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    extracted_time.nanosec = static_cast<uint32_t>(
      gxf_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[ArgusCameraNode] Failed to get timestamp");
  }

  return extracted_time;
}

tf2::Transform ArgusCameraNode::rosTransformFromGxfTransform(
  const nvidia::gxf::Handle<nvidia::gxf::Pose3D> & gxf_pose)
{
  const tf2::Vector3 translation(
    gxf_pose->translation[0],
    gxf_pose->translation[1],
    gxf_pose->translation[2]);
  // tf2::Matrix3x3 is row major
  const tf2::Matrix3x3 rot_mat = {
    gxf_pose->rotation[0],
    gxf_pose->rotation[1],
    gxf_pose->rotation[2],
    gxf_pose->rotation[3],
    gxf_pose->rotation[4],
    gxf_pose->rotation[5],
    gxf_pose->rotation[6],
    gxf_pose->rotation[7],
    gxf_pose->rotation[8]};

  const tf2::Transform transform(
    rot_mat,
    translation);

  // Shifting extrinsics from gxf to ros conventions
  // Extrinsics GXF Convention => right_pose_left AKA left_wrt_right
  // Extrinsics ROS Convetion =>  left_pose_right AKA right_wrt_left
  tf2::Transform transform_inverse = transform.inverse();
  return transform_inverse;
}


void ArgusCameraNode::fillTransformMsgFromTransform(
  geometry_msgs::msg::TransformStamped & transform_msg,
  const tf2::Transform & transform)
{
  transform_msg.transform.translation.x = transform.getOrigin().getX();
  transform_msg.transform.translation.y = transform.getOrigin().getY();
  transform_msg.transform.translation.z = transform.getOrigin().getZ();
  transform_msg.transform.rotation.x = transform.getRotation().getX();
  transform_msg.transform.rotation.y = transform.getRotation().getY();
  transform_msg.transform.rotation.z = transform.getRotation().getZ();
  transform_msg.transform.rotation.w = transform.getRotation().getW();
}

void ArgusCameraNode::ArgusCameraInfoCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg,
  const std::string parent_frame, const std::string child_frame,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);

  geometry_msgs::msg::TransformStamped transform_stamped;
  msg.frame_id = child_frame;
  transform_stamped.header.frame_id = parent_frame;
  transform_stamped.child_frame_id = child_frame;
  transform_stamped.header.stamp = timestampFromGxfMessage(msg_entity);

  // Fill in CameraModel if camera info is provided
  if (camera_info != nullptr) {
    nvidia::isaac_ros::nitros::copy_ros_to_gxf_camera_info(*camera_info, msg_entity);
  }
  if (tf_was_published_.find(transform_stamped.child_frame_id) == tf_was_published_.end()) {
    // Extract camera extrinsics
    auto camera_optical_pose_rig_optical =
      msg_entity->get<nvidia::gxf::Pose3D>(GXF_EXTRINSICS_NAME);
    if (!camera_optical_pose_rig_optical) {
      std::stringstream error_msg;
      error_msg << "[ArgusCameraNode] Failed pose get Pose3D object from message entity: " <<
        GxfResultStr(camera_optical_pose_rig_optical.error());
      RCLCPP_ERROR(
        get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    tf2::Transform rig_optical_pose_camera_optical =
      rosTransformFromGxfTransform(camera_optical_pose_rig_optical.value());
    // Computing camera_link->optical tf transform
    tf2::Transform rig_body_pose_camera_optical =
      cam_link_pose_optical_ * rig_optical_pose_camera_optical;
    fillTransformMsgFromTransform(transform_stamped, rig_body_pose_camera_optical);

    tf_broadcaster_->sendTransform(transform_stamped);
    tf_was_published_.insert(transform_stamped.child_frame_id);
  }
}

// perform the same computation as in stereo_info_creator.py, used only in the case of overrides
tf2::Transform ArgusCameraNode::computeLeftOpticalPoseRightOptical(
  const sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info,
  const sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info)
{
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> right_r_map(
    right_camera_info->r.data());
  Eigen::Matrix3d right_r = right_r_map;

  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> left_r_map(
    left_camera_info->r.data());
  Eigen::Matrix3d left_r = left_r_map;

  Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> right_p_map(
    right_camera_info->p.data());
  Eigen::Matrix<double, 3, 4> right_p = right_p_map;

  double f_x = right_camera_info->p[0];

  // Compute the translation vector t = (1/f_x) * right_p's fourth column
  Eigen::Vector3d t = (1.0 / f_x) * right_p.col(3).head<3>();

  // Helper function to create a 4x4 transformation matrix
  auto matrix4x4 = [](const Eigen::Matrix3d * rotation,
      const Eigen::Vector3d * translation) -> Eigen::Matrix4d {
      Eigen::Matrix4d pose_3d = Eigen::Matrix4d::Identity();
      if (rotation != nullptr) {
        pose_3d.block<3, 3>(0, 0) = *rotation;
      }
      if (translation != nullptr) {
        pose_3d.block<3, 1>(0, 3) = *translation;
      }
      return pose_3d;
    };

  // Compute the three transformation matrices
  Eigen::Matrix4d right_T_rectified_right =
    matrix4x4(&right_r, nullptr).transpose();
  Eigen::Matrix4d rectified_right_T_rectified_left =
    matrix4x4(nullptr, &t);
  Eigen::Matrix4d rectified_left_T_left = matrix4x4(&left_r, nullptr);

  // Compute the final transformation matrix
  Eigen::Matrix4d right_T_left = right_T_rectified_right *
    rectified_right_T_rectified_left *
    rectified_left_T_left;

  // Extract rotation and translation components
  tf2::Matrix3x3 rotation(
    right_T_left(0, 0), right_T_left(0, 1), right_T_left(0, 2),
    right_T_left(1, 0), right_T_left(1, 1), right_T_left(1, 2),
    right_T_left(2, 0), right_T_left(2, 1), right_T_left(2, 2));

  tf2::Vector3 translation(
    right_T_left(0, 3), right_T_left(1, 3), right_T_left(2, 3));

  tf2::Transform transform(rotation, translation);

  // Take inverse to match ROS conventions
  return transform.inverse();
}

void ArgusCameraNode::ArgusStereoCameraInfoCallback(
  const gxf_context_t context, nitros::NitrosTypeBase & msg,
  const std::string rig_frame, const std::string camera_frame,
  const sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info,
  const sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info,
  const StereoCameraSide side)
{
  auto msg_entity = nvidia::gxf::Entity::Shared(context, msg.handle);
  const std::string optical_frame_name = camera_frame + "_optical";
  msg.frame_id = optical_frame_name;

  if (side == LEFT && left_camera_info != nullptr && right_camera_info != nullptr) {
    nvidia::isaac_ros::nitros::copy_ros_to_gxf_camera_info(*left_camera_info, msg_entity);
  }
  if (side == RIGHT && right_camera_info != nullptr && left_camera_info != nullptr) {
    nvidia::isaac_ros::nitros::copy_ros_to_gxf_camera_info(*right_camera_info, msg_entity);
    if (!right_camera_override_tf_set_) {
      right_camera_pose_rig_override_ = computeLeftOpticalPoseRightOptical(
        left_camera_info,
        right_camera_info);
      right_camera_override_tf_set_ = true;
    }
  }

  geometry_msgs::msg::TransformStamped camera_transform_stamped;
  camera_transform_stamped.header.frame_id = rig_frame;
  camera_transform_stamped.child_frame_id = camera_frame;
  camera_transform_stamped.header.stamp = timestampFromGxfMessage(msg_entity);

  geometry_msgs::msg::TransformStamped camera_optical_transform_stamped;
  camera_optical_transform_stamped.header.frame_id = camera_frame;
  camera_optical_transform_stamped.child_frame_id = optical_frame_name;
  camera_optical_transform_stamped.header.stamp = timestampFromGxfMessage(msg_entity);

  if (tf_was_published_.find(camera_transform_stamped.child_frame_id) ==
    tf_was_published_.end())
  {
    // Extract camera extrinsics
    auto camera_optical_pose_rig_optical =
      msg_entity->get<nvidia::gxf::Pose3D>(GXF_EXTRINSICS_NAME);
    if (!camera_optical_pose_rig_optical) {
      std::stringstream error_msg;
      error_msg << "[ArgusCameraNode] Failed pose get Pose3D object from message entity: " <<
        GxfResultStr(camera_optical_pose_rig_optical.error());
      RCLCPP_ERROR(
        get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    tf2::Transform rig_optical_pose_camera_optical;
    if (right_camera_override_tf_set_) {
      rig_optical_pose_camera_optical = right_camera_pose_rig_override_;
    } else {
      rig_optical_pose_camera_optical = rosTransformFromGxfTransform(
        camera_optical_pose_rig_optical.value());
    }

    // Computing camera_link->optical tf transform
    tf2::Transform rig_body_pose_camera_optical =
      cam_link_pose_optical_ * rig_optical_pose_camera_optical;

    tf2::Transform rig_pose_camera = rig_body_pose_camera_optical *
      cam_link_pose_optical_.inverse();

    fillTransformMsgFromTransform(camera_transform_stamped, rig_pose_camera);
    tf_broadcaster_->sendTransform(camera_transform_stamped);
    tf_was_published_.insert(camera_transform_stamped.child_frame_id);

    fillTransformMsgFromTransform(camera_optical_transform_stamped, cam_link_pose_optical_);
    tf_broadcaster_->sendTransform(camera_optical_transform_stamped);
    tf_was_published_.insert(camera_optical_transform_stamped.child_frame_id);
  }

  geometry_msgs::msg::TransformStamped imu_transform_stamped;
  imu_transform_stamped.header.frame_id = rig_frame;
  imu_transform_stamped.child_frame_id = rig_frame + "_imu";
  imu_transform_stamped.header.stamp = timestampFromGxfMessage(msg_entity);

  // If there is IMU info attached to the message then publish those as well
  if (tf_was_published_.find(imu_transform_stamped.child_frame_id) == tf_was_published_.end()) {
    auto maybe_imu_pose_rig_optical = msg_entity->get<nvidia::gxf::Pose3D>(IMU_EXTRINSICS_NAME);
    if (maybe_imu_pose_rig_optical) {
      tf2::Transform rig_optical_pose_imu =
        rosTransformFromGxfTransform(maybe_imu_pose_rig_optical.value());

      // Computing camera_link->imu tf transform
      tf2::Transform rig_pose_imu =
        cam_link_pose_optical_ * rig_optical_pose_imu;

      fillTransformMsgFromTransform(imu_transform_stamped, rig_pose_imu);

      tf_broadcaster_->sendTransform(imu_transform_stamped);
      tf_was_published_.insert(imu_transform_stamped.child_frame_id);
    }
  }
}

void ArgusCameraNode::preLoadGraphCallback()
{
}

void ArgusCameraNode::postLoadGraphCallback()
{
  RCLCPP_INFO(get_logger(), "[ArgusCameraNode] postLoadGraphCallback().");
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "camera_id", camera_id_);
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "module_id", module_id_);
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "mode", mode_);
  getNitrosContext().setParameterInt32(
    "argus_camera", "nvidia::isaac::ArgusCamera", "fsync_type", fsync_type_);
  getNitrosContext().setParameterBool(
    "argus_camera", "nvidia::isaac::ArgusCamera", "use_hw_timestamp", use_hw_timestamp_);
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
      auto camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>(
        sensor_msgs::msg::CameraInfo(cinfo.getCameraInfo()));
      if (camera_info->p[7] != 0.0f || camera_info->p[11] != 0.0) {
        std::stringstream warning_msg;
        warning_msg << "[ArgusCameraNode] Camera info " << camera_name <<
          " has non-zero values in p[7] or p[11] which are expected to be zero." <<
          " See https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html";
        RCLCPP_WARN(
          get_logger(), warning_msg.str().c_str());
      }
      return camera_info;
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
