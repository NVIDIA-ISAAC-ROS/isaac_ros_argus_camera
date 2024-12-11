# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict

from isaac_ros_examples import IsaacROSLaunchFragment
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class IsaacROSArgusDepthLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': 960, 'height': 576},
            'camera_frame': 'stereo_camera',
            'focal_length': {
                'f_x': 480,  # Approximation - most Hawk cameras should be close to this value
                'f_y': 460   # Approximation - most Hawk cameras should be close to this value
            }
        }

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        module_id = LaunchConfiguration('module_id')
        return {
            'camera_node': ComposableNode(
                name='argus_stereo',
                package='isaac_ros_argus_camera',
                plugin='nvidia::isaac_ros::argus::ArgusStereoNode',
                namespace='',
                parameters=[{'module_id': module_id}]
            ),

            'left_resize_node': ComposableNode(
                name='left_resize_node',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'output_width': 960,
                    'output_height': 576,
                }],
                remappings=[
                    ('image', 'left/image_raw'),
                    ('camera_info', 'left/camera_info'),
                    ('resize/image', 'left/image_resize'),
                    ('resize/camera_info', 'left/camera_info_resize')
                ]
            ),

            'right_resize_node': ComposableNode(
                name='right_resize_node',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'output_width': 960,
                    'output_height': 576,
                }],
                remappings=[
                    ('image', 'right/image_raw'),
                    ('camera_info', 'right/camera_info'),
                    ('resize/image', 'right/image_resize'),
                    ('resize/camera_info', 'right/camera_info_resize')
                ]
            ),

            'rectify_left_node': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_left',
                namespace='',
                parameters=[{
                    'output_width': 960,
                    'output_height': 576,
                }],
                remappings=[
                    ('/image_raw', 'left/image_resize'),
                    ('/camera_info', '/left/camera_info_resize'),
                ]
            ),
            'rectify_right_node': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_right',
                namespace='',
                parameters=[{
                    'output_width': 960,
                    'output_height': 576,
                }],
                remappings=[
                    ('/image_raw', '/right/image_resize'),
                    ('/camera_info', '/right/camera_info_resize'),
                    ('/image_rect', '/right/image_rect'),
                    ('/camera_info_rect', '/right/camera_info_rect'),
                ]
            ),
            'disparity_node': ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
                name='disparity',
                namespace='',
                remappings=[('left/image_rect', 'image_rect'),
                            ('left/camera_info', 'camera_info_rect'),
                            ('right/image_rect', 'right/image_rect'),
                            ('right/camera_info', 'right/camera_info_rect')]
            ),
            'disparity_to_depth_node': ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
                name='disparity_to_depth',
                namespace=''
            )
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> \
            Dict[str, launch.actions.OpaqueFunction]:
        return {
            'module_id': DeclareLaunchArgument(
                'module_id',
                default_value='0',
                description='Index specifying the stereo camera module to use.'
            )}


def generate_launch_description():
    argus_stereo_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='argus_stereo_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=IsaacROSArgusDepthLaunchFragment
        .get_composable_nodes().values(),
        output='screen'
    )

    return launch.LaunchDescription(
        [argus_stereo_container] +
        IsaacROSArgusDepthLaunchFragment.get_launch_actions().values())
