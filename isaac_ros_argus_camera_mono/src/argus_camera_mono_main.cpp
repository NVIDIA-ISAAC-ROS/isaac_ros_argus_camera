/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_argus_camera_mono/argus_camera_mono_node.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions argus_monocular_options;
  argus_monocular_options.arguments(
  {
    "--ros-args",
    "-r", "__node:=argus_monocular"
  });
  auto argus_monocular_node = std::make_shared<isaac_ros::argus::MonocularNode>(
    argus_monocular_options);
  exec.add_node(argus_monocular_node);

  // Spin with all the components loaded
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
