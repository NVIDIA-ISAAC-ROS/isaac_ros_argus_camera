# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import pathlib
import socket
import subprocess
import time

import cv2
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import pytest
import rclpy

from sensor_msgs.msg import Image

VISUALIZE = False


@pytest.mark.rostest
def generate_test_description():
    device_id = -1
    command = 'if ls /dev/video* 1> /dev/null 2>&1;  \
               then echo Device Found; \
               else echo Device Not Found; fi'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)

    if (result.stdout.strip() == 'Device Found'):
        IsaacArgusStereoNodeTest.skip_test = False

        # restart the argus server
        s = socket.socket(socket.AF_UNIX)
        s.connect('/tmp/argus_restart_socket')
        s.send(b'RESTART_SERVICE')
        s.close()
        time.sleep(1)

        argus_stereo_node = ComposableNode(
            name='argus_stereo',
            package='isaac_ros_argus_camera',
            plugin='nvidia::isaac_ros::argus::ArgusStereoNode',
            namespace=IsaacArgusStereoNodeTest.generate_namespace(),
            parameters=[{'module_id': device_id}]
        )

        left_rectify_node = ComposableNode(
            name='left_rectify_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            namespace=IsaacArgusStereoNodeTest.generate_namespace(),
            parameters=[{
                'output_width': 1920,
                'output_height': 1200
            }],
            remappings=[
                ('image_raw', 'left/image_raw'),
                ('camera_info', 'left/camera_info'),
                ('image_rect', 'left/image_rect'),
                ('camera_info_rect', 'left/camera_info_rect')
            ]
        )

        right_rectify_node = ComposableNode(
            name='right_rectify_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            namespace=IsaacArgusStereoNodeTest.generate_namespace(),
            parameters=[{
                'output_width': 1920,
                'output_height': 1200,
            }],
            remappings=[
                ('image_raw', 'right/image_raw'),
                ('camera_info', 'right/camera_info'),
                ('image_rect', 'right/image_rect'),
                ('camera_info_rect', 'right/camera_info_rect')
            ]
        )

        left_oss_rectify_node = ComposableNode(
            name='left_oss_rectify_node',
            package='image_proc',
            plugin='image_proc::RectifyNode',
            namespace=IsaacArgusStereoNodeTest.generate_namespace(),
            parameters=[{
                'width': 1920,
                'height': 1200,
                'use_scale': False
            }],
            remappings=[
                ('image', 'left/image_raw'),
                ('camera_info', 'left/camera_info'),
                ('image_rect', 'left/image_rect_oss')
            ]
        )

        right_oss_rectify_node = ComposableNode(
            name='right_oss_rectify_node',
            package='image_proc',
            plugin='image_proc::RectifyNode',
            namespace=IsaacArgusStereoNodeTest.generate_namespace(),
            parameters=[{
                'width': 1920,
                'height': 1200,
                'use_scale': False
            }],
            remappings=[
                ('image', 'right/image_raw'),
                ('camera_info', 'right/camera_info'),
                ('image_rect', 'right/image_rect_oss')
            ]
        )

        return IsaacArgusStereoNodeTest.generate_test_description([
            ComposableNodeContainer(
                name='argus_stereo_container',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[argus_stereo_node,
                                              left_rectify_node,
                                              right_rectify_node,
                                              left_oss_rectify_node,
                                              right_oss_rectify_node
                                              ],
                namespace=IsaacArgusStereoNodeTest.generate_namespace(),
                output='screen'
            )
        ])
    else:
        IsaacArgusStereoNodeTest.skip_test = True
        return IsaacArgusStereoNodeTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacArgusStereoNodeTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))
    skip_test = False

    def test_image_capture(self):
        """
        Test image capture from argus stereo camera.

        Test that checks if the OSS rectified images and the Isaaac ROS
        rectified images are almost equal
        """
        if self.skip_test:
            self.skipTest('No camera detected! Skipping test.')
        else:
            TIMEOUT = 10
            received_messages = []

            self.create_exact_time_sync_logging_subscribers(
                [('left/image_rect_oss', Image), ('right/image_rect_oss', Image),
                 ('left/image_rect', Image), ('right/image_rect', Image)],
                received_messages,
                accept_multiple_messages=True)
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if len(received_messages) > 0:
                    done = True
                    break
            self.assertTrue(done, 'Appropriate output not received')
            for received_message in received_messages:
                image_left_rect_oss = self.bridge.imgmsg_to_cv2(received_message[0])
                image_right_rect_oss = self.bridge.imgmsg_to_cv2(received_message[1])
                image_right_rect = self.bridge.imgmsg_to_cv2(received_message[1])
                image_left_rect = self.bridge.imgmsg_to_cv2(received_message[2])
                if (VISUALIZE):
                    cv2.imwrite('image_left_rect.png', image_left_rect)
                    cv2.imwrite('image_right_rect.png', image_right_rect)
                    cv2.imwrite('image_left_rect_oss.png', image_left_rect_oss)
                    cv2.imwrite('image_right_rect_oss.png', image_right_rect_oss)
                    cv2.imwrite('image_left_diff.png',
                                cv2.absdiff(image_left_rect, image_left_rect_oss))
                    cv2.imwrite('image_right_diff.png',
                                cv2.absdiff(image_right_rect, image_right_rect_oss))
                self.assertImagesEqual(image_left_rect, image_left_rect_oss, 0.00001)
                self.assertImagesEqual(image_right_rect, image_right_rect_oss, 0.00001)
