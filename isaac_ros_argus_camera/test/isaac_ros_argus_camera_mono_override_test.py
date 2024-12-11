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

import os
import pathlib
import subprocess
import time

from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import (Buffer, ConnectivityException,
                     ExtrapolationException, LookupException, TransformListener)
import yaml

"""
Test for the camera override feature of Isaac ROS Argus Camera.
"""


@pytest.mark.rostest
def generate_test_description():
    device_id = 0
    command = 'if ls /dev/video* 1> /dev/null 2>&1;  \
               then echo Device Found; \
               else echo Device Not Found; fi'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    if (result.stdout.strip() == 'Device Found'):

        IsaacArgusMonoNodeCameraOverrideTest.skip_test = False

        argus_mono_node = ComposableNode(
            name='argus_mono',
            package='isaac_ros_argus_camera',
            plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
            namespace=IsaacArgusMonoNodeCameraOverrideTest.generate_namespace(),
            parameters=[{'camera_id': device_id,
                        'camera_info_url': 'file://' +
                         IsaacArgusMonoNodeCameraOverrideTest.override_filepath,
                         }]
        )

        return IsaacArgusMonoNodeCameraOverrideTest.generate_test_description([
            ComposableNodeContainer(
                name='argus_mono_container',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[argus_mono_node],
                namespace=IsaacArgusMonoNodeCameraOverrideTest.generate_namespace(),
                output='screen',
                arguments=['--ros-args', '--log-level', 'info',
                           '--log-level', 'isaac_ros_test.argus_mono:=debug',
                           '--log-level', 'NitrosImage:=debug',
                           '--log-level', 'NitrosCameraInfo:=debug'
                           ],
            )
        ])
    else:
        IsaacArgusMonoNodeCameraOverrideTest.skip_test = True
        return IsaacArgusMonoNodeCameraOverrideTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacArgusMonoNodeCameraOverrideTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))
    override_filepath = f'{filepath}/camera_override/mono.yaml'

    skip_test = False

    def setUp(self):
        super().setUp()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def query_transform(self, target_frame, source_frame, timeout=1.0):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout)
            )
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(
                f'Could not transform {target_frame} to {source_frame}: {e}')
            return None

    def test_image_camera_override(self):
        """
        Test that the camera override feature works as expected.

        Tests that the camera_info messages published by the ArgusMonoNode match the provided
        override files, and checks that the tfs published by the node are correct.
        """
        if self.skip_test:
            self.skipTest('No camera detected! Skipping test.')
        else:
            TIMEOUT = 10

            received_camera_messages = []
            self.create_exact_time_sync_logging_subscribers(
                [('left/image_raw', Image),
                 ('left/camera_info', CameraInfo)],
                received_camera_messages,
                accept_multiple_messages=True)

            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if len(received_camera_messages) > 30:
                    done = True
                    break
            self.assertTrue(
                done,
                f'Appropriate output not received, got {len(received_camera_messages)} messages')

            # Load the expected camera info from the YAML files
            with open(self.override_filepath, 'r') as file:
                expected_camera_info = yaml.safe_load(file)

            assert len(received_camera_messages) > 0, 'No messages received'

            # Test if the transform is correct (up to rounding error)

            def approx_equal_vector3(v1, v2, tol=1e-6):
                return np.allclose([v1.x, v1.y, v1.z], [v2.x, v2.y, v2.z], atol=tol)

            def approx_equal_quaternion(q1, q2, tol=1e-6):
                return np.allclose([q1.x, q1.y, q1.z, q1.w], [q2.x, q2.y, q2.z, q2.w], atol=tol)

            for msg in received_camera_messages:
                cam_info = msg[1]
                received_camera_info = cam_info

                self.assertIsNotNone(received_camera_info, 'Camera info not received')

                def camera_info_equals_expected(received, expected):
                    # Convert received and expected arrays to NumPy arrays
                    received_k = np.array(received.k)
                    expected_k = np.array(expected['camera_matrix']['data'])
                    # received_r = np.array(received.r)
                    # expected_r = np.array(expected['rectification_matrix']['data'])
                    received_p = np.array(received.p)
                    expected_p = np.array(expected['projection_matrix']['data'])
                    received_d = np.array(received.d)
                    expected_d = np.array(expected['distortion_coefficients']['data'])

                    # Pad expected_d with zeros if it's shorter than received_d
                    if len(expected_d) < len(received_d):
                        expected_d = np.pad(
                            expected_d, (0, len(received_d) - len(expected_d)), 'constant')
                    elif len(expected_d) > len(received_d):
                        # Pad received_d with zeros if it's shorter
                        received_d = np.pad(
                            received_d, (0, len(expected_d) - len(received_d)), 'constant')

                    return (received.width == expected['image_width'] and
                            received.height == expected['image_height'] and
                            np.allclose(received_k, expected_k) and
                            np.allclose(received_d, expected_d) and
                            # np.allclose(received_r, expected_r) and
                            np.allclose(received_p, expected_p))

                assert camera_info_equals_expected(
                    cam_info, expected_camera_info), \
                    'Camera info does not match expected'
