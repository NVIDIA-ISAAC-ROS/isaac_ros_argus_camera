# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'module_id',
            default_value='-1',
            description='Index specifying the stereo camera module to use.'),
        DeclareLaunchArgument(
            'left_camera_info_url',
            default_value='',
            description='URL for the left camera info file.'
        ),
        DeclareLaunchArgument(
            'right_camera_info_url',
            default_value='',
            description='URL for the right camera info file.'
        ),
    ]
    module_id = LaunchConfiguration('module_id')
    left_camera_info_url = LaunchConfiguration('left_camera_info_url')
    right_camera_info_url = LaunchConfiguration('right_camera_info_url')

    argus_stereo_node = ComposableNode(
        name='argus_stereo',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusStereoNode',
        namespace='',
        parameters=[{'module_id': module_id,
                     'left_camera_info_url': left_camera_info_url,
                     'right_camera_info_url': right_camera_info_url}],
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

    return launch.LaunchDescription(launch_args + [argus_stereo_container])
