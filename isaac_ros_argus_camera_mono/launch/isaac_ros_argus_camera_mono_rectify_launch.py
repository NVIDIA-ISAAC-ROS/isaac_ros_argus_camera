# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up argus monocular camera node with rectify node."""
    rectify_node = ComposableNode(
        name='isaac_ros_rectify',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        remappings=[('image', 'image_color')])
    argus_camera_mono_node = Node(
        package='isaac_ros_argus_camera_mono',
        executable='isaac_ros_argus_camera_mono',
        parameters=[{
                'sensor': 0,
                'device': 0,
                'output_encoding': 'rgb8'
        }],
        remappings=[('image_raw', 'image_color')]
    )
    argus_camera_mono_container = ComposableNodeContainer(
        name='argus_camera_mono_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node
        ],
        output='screen'
    )
    return launch.LaunchDescription([argus_camera_mono_container, argus_camera_mono_node])
