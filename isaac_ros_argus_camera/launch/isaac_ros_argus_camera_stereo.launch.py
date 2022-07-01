# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    argus_stereo_node = ComposableNode(
        name='argus_stereo',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusStereoNode',
        namespace='',
    )

    argus_stereo_container = ComposableNodeContainer(
            name='argus_stereo_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[argus_stereo_node],
            namespace='',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        )

    return launch.LaunchDescription([argus_stereo_container])
