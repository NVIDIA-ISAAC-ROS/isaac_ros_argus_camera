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
import socket
import subprocess
import time

from geometry_msgs.msg import Quaternion, Vector3
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
    device_id = -1
    command = 'if ls /dev/video* 1> /dev/null 2>&1;  \
               then echo Device Found; \
               else echo Device Not Found; fi'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    if (result.stdout.strip() == 'Device Found'):
        IsaacArgusStereoNodeCameraOverrideTest.skip_test = False

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
            namespace=IsaacArgusStereoNodeCameraOverrideTest.generate_namespace(),
            parameters=[{'module_id': device_id,
                        'left_camera_info_url': 'file://' +
                         IsaacArgusStereoNodeCameraOverrideTest.left_override_filepath,
                         'right_camera_info_url': 'file://' +
                         IsaacArgusStereoNodeCameraOverrideTest.right_override_filepath
                         }]
        )

        return IsaacArgusStereoNodeCameraOverrideTest.generate_test_description([
            ComposableNodeContainer(
                name='argus_stereo_container',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[argus_stereo_node],
                namespace=IsaacArgusStereoNodeCameraOverrideTest.generate_namespace(),
                output='screen',
                arguments=['--ros-args', '--log-level', 'info',
                           '--log-level', 'isaac_ros_test.argus_stereo:=debug',
                           '--log-level', 'NitrosImage:=debug',
                           '--log-level', 'NitrosCameraInfo:=debug'
                           ],
            )
        ])
    else:
        IsaacArgusStereoNodeCameraOverrideTest.skip_test = True
        return IsaacArgusStereoNodeCameraOverrideTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacArgusStereoNodeCameraOverrideTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))
    left_override_filepath = f'{filepath}/camera_override/left.yaml'
    right_override_filepath = f'{filepath}/camera_override/right.yaml'

    expected_translation = Vector3(x=-0.1496265979, y=0.0000348223, z=0.0004907123)
    expected_rotation = Quaternion(x=0.001225584, y=0.0016536812, z=-0.0000114609, w=0.9999978816)

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

        Tests that the camera_info messages published by the ArgusStereoNode match the provided
        override files, and checks that the tfs published by the node are correct.
        """
        if self.skip_test:
            self.skipTest('No camera detected! Skipping test.')
        else:
            TIMEOUT = 10

            received_camera_messages = []
            self.create_exact_time_sync_logging_subscribers(
                [('left/image_raw', Image), ('right/image_raw', Image),
                 ('left/camera_info', CameraInfo), ('right/camera_info', CameraInfo)],
                received_camera_messages,
                accept_multiple_messages=True)

            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if len(received_camera_messages) > 30:
                    done = True
                    break
            self.assertTrue(done, 'Appropriate output not received')

            # Load the expected camera info from the YAML files
            with open(self.left_override_filepath, 'r') as left_file:
                expected_left_camera_info = yaml.safe_load(left_file)

            with open(self.right_override_filepath, 'r') as right_file:
                expected_right_camera_info = yaml.safe_load(right_file)

            assert len(received_camera_messages) > 0, 'No messages received'

            # Test if the transform is correct (up to rounding error)

            def approx_equal_vector3(v1, v2, tol=1e-6):
                return np.allclose([v1.x, v1.y, v1.z], [v2.x, v2.y, v2.z], atol=tol)

            def approx_equal_quaternion(q1, q2, tol=1e-6):
                return np.allclose([q1.x, q1.y, q1.z, q1.w], [q2.x, q2.y, q2.z, q2.w], atol=tol)

            tf_message = self.query_transform(
                'stereo_camera_right_optical', 'stereo_camera_left_optical')
            assert tf_message is not None, 'Could not get transform'

            tf = tf_message.transform

            assert approx_equal_vector3(
                tf.translation, self.expected_translation), 'Translation does not match expected'
            assert approx_equal_quaternion(
                tf.rotation, self.expected_rotation), 'Rotation does not match expected'

            for msg in received_camera_messages:
                left_cam_info = msg[2]
                right_cam_info = msg[3]

                received_left_camera_info = left_cam_info
                received_right_camera_info = right_cam_info

                self.assertIsNotNone(received_left_camera_info, 'Left camera info not received')
                self.assertIsNotNone(received_right_camera_info, 'Right camera info not received')

                def camera_info_equals_expected(received, expected):
                    # Convert received and expected arrays to NumPy arrays
                    received_k = np.array(received.k)
                    expected_k = np.array(expected['camera_matrix']['data'])
                    received_r = np.array(received.r)
                    expected_r = np.array(expected['rectification_matrix']['data'])
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
                            np.allclose(received_r, expected_r) and
                            np.allclose(received_p, expected_p))

                assert camera_info_equals_expected(
                    left_cam_info, expected_left_camera_info), \
                    'Left camera info does not match expected'
                assert camera_info_equals_expected(right_cam_info, expected_right_camera_info), \
                    'Right camera info does not match expected'
