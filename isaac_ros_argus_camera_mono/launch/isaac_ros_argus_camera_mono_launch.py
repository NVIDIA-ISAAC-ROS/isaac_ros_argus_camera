# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited

import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='isaac_ros_argus_camera_mono',
            executable='isaac_ros_argus_camera_mono',
            parameters=[{
                'sensor': 0,
                'device': 0,
                'output_encoding': 'rgb8'
            }]
        ),
        Node(
            package='image_view',
            executable='image_saver',
            remappings=[
                ('/image', '/image_raw')
            ],
            parameters=[{
                'filename_format': 'image.jpg'
            }]
        )
    ])
