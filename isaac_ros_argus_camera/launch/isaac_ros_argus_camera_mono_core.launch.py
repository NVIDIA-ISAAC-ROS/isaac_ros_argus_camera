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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class IsaacROSArgusMonoLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': 1920, 'height': 1200},
            'camera_frame': 'left_cam',
            'focal_length': {
                'f_x': 480.0,   # Approximation - most Hawk cameras should be close to this value
                'f_y': 460.0    # Approximation - most Hawk cameras should be close to this value
            }
        }

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        return {
            'camera_node': ComposableNode(
                name='argus_mono',
                package='isaac_ros_argus_camera',
                plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
                remappings=[
                    ('left/image_raw', 'image_raw'),
                    ('left/camera_info', 'camera_info')
                ]
            )
        }


def generate_launch_description():
    argus_mono_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='argus_mono_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=IsaacROSArgusMonoLaunchFragment
        .get_composable_nodes().values(),
        output='screen'
    )

    return launch.LaunchDescription(
        [argus_mono_container] + IsaacROSArgusMonoLaunchFragment.get_launch_actions().values())
