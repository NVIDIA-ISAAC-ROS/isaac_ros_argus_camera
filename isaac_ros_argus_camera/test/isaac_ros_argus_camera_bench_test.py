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

from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import launch_testing
import numpy
import pytest
import rclpy

from sensor_msgs.msg import CameraInfo, Image

SKIP_TEST = True
EXPECTED_FPS = 30.0
SEC_TO_MS = 10**3
MS_TO_NS = 10**6
MAX_ROW_WIDTH = 70
JITTER_TOLERANCE = 5

EXCEED_TOLERANCE_PERCENT_LIMIT = 5
FPS_ACCEPTABLE_VARIANCE = 2

"""
Bench Test for Isaac ROS Argus

Test including:
- Performance
    1. FPS (Sustanable 30 FPS)
    2. Jitter (Less than 5 ms)
- System Usage
    1. CPU Usage
    2. GPU Usage
- Quality
    1. left/right timestamp match (exact sync)
"""


@pytest.mark.rostest
def generate_test_description():
    device_id = -1
    command = 'if ls /dev/video* 1> /dev/null 2>&1;  \
               then echo Device Found; \
               else echo Device Not Found; fi'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)

    if (result.stdout.strip() == 'Device Found'):
        IsaacArgusBenchTest.skip_test = False

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
            namespace=IsaacArgusBenchTest.generate_namespace(),
            parameters=[{'module_id': device_id}]
        )

        return IsaacArgusBenchTest.generate_test_description([
            ComposableNodeContainer(
                name='argus_stereo_container',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[argus_stereo_node],
                namespace=IsaacArgusBenchTest.generate_namespace(),
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
            )
        ])
    else:
        IsaacArgusBenchTest.skip_test = True
        return IsaacArgusBenchTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacArgusBenchTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))
    skip_test = False

    def test_image_capture(self):
        """
        Test camera streams from argus stereo camera on bench unit.

        Report FPS, Jitter for each topic, and the left/right matching statistics.
        """
        if self.skip_test:
            self.skipTest('No camera detected! Skipping test.')
        else:
            TIMEOUT = 30
            received_messages = []

            topics = ['left/image_raw', 'right/image_raw', 'left/camera_info', 'right/camera_info']
            self.generate_namespace_lookup(topics)

            self.create_exact_time_sync_logging_subscribers(
                [('left/image_raw', Image), ('right/image_raw', Image),
                 ('left/camera_info', CameraInfo), ('right/camera_info', CameraInfo)],
                received_messages,
                accept_multiple_messages=True)

            end_time = time.time() + TIMEOUT
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.01))

            self.assertTrue(len(received_messages) > 0, 'Did not receive synced messages')

            # Record the message statistic
            exceed_tolerance_count = 0
            timestamps_ms = []
            for message in received_messages:
                self.assertTrue(message[0].header.stamp ==
                                message[1].header.stamp and
                                message[1].header.stamp ==
                                message[2].header.stamp and
                                message[2].header.stamp ==
                                message[3].header.stamp,
                                'Time stamps of all images and camera infos are not equal')
                self.assertTrue(message[0].header.frame_id ==
                                message[2].header.frame_id,
                                'Frame IDs of all images and camera infos are not equal')
                self.assertTrue(message[1].header.frame_id ==
                                message[3].header.frame_id,
                                'Frame IDs of all images and camera infos are not equal')

                message_timestamp_ms = \
                    message[0].header.stamp.sec * SEC_TO_MS + \
                    message[0].header.stamp.nanosec / MS_TO_NS
                timestamps_ms.append(message_timestamp_ms)

            expected_diff = SEC_TO_MS / EXPECTED_FPS
            jitters = numpy.abs(numpy.diff(timestamps_ms))
            for index in range(len(jitters)):
                jitter = abs(jitters[index] - expected_diff)
                if (jitter > JITTER_TOLERANCE):
                    exceed_tolerance_count += 1
                jitters[index] = jitter

            duration = timestamps_ms[-1] - timestamps_ms[0]
            if duration > 0:
                frame_rate = \
                    len(timestamps_ms) / (duration / SEC_TO_MS)
            else:
                self.node.get_logger().warning(
                    'Could not compute MEAN_FRAME_RATE due to an invalid value of'
                    f'RECEIVED_DURATION = {duration}')
                frame_rate = 0

            exceed_tolerance_percent = exceed_tolerance_count * 100.0 / len(timestamps_ms)

            # Report message statistics
            heading = 'Argus Camera Bench Test Statistics'
            self.node.get_logger().info('+-{}-+'.format('-' * MAX_ROW_WIDTH))
            self.node.get_logger().info(
                '| {:^{width}} |'.format(heading, width=MAX_ROW_WIDTH))
            self.node.get_logger().info('+-{}-+'.format('-' * MAX_ROW_WIDTH))
            self.node.get_logger().info(
                f'# of Synced Frame Groups: {len(timestamps_ms)}')
            self.node.get_logger().info(
                f'Delta between First & Last Received Frames (ms): '
                f'{float(duration)}')
            self.node.get_logger().info(
                f'Mean Frame Rate (fps) : {frame_rate}')
            self.node.get_logger().info(
                f'Max. Jitter (ms) : {float(numpy.max(jitters))}')
            self.node.get_logger().info(
                f'Min. Jitter (ms) : {float(numpy.min(jitters))}')
            self.node.get_logger().info(
                f'Mean. Jitter (ms) : {float(numpy.mean(jitters))}')
            self.node.get_logger().info(
                f'% of Frame Groups Out Of Jitter Tolerance ({JITTER_TOLERANCE} ms) : '
                f'{exceed_tolerance_percent}')
            self.node.get_logger().info('+-{}-+'.format('-' * MAX_ROW_WIDTH))

            # Raise error if not meet the performance requirement
            self.assertTrue(exceed_tolerance_percent < EXCEED_TOLERANCE_PERCENT_LIMIT,
                            f'{exceed_tolerance_percent}% of jitters exceed the tolerance')
            self.assertTrue(frame_rate > (EXPECTED_FPS - FPS_ACCEPTABLE_VARIANCE),
                            f'Captured Frame rate {frame_rate} is less than expected')
            self.assertTrue(frame_rate < (EXPECTED_FPS + FPS_ACCEPTABLE_VARIANCE),
                            f'Captured frame rate {frame_rate} is larger than expected')
