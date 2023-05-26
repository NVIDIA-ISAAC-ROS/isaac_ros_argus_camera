# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


from collections import defaultdict
import os
import pathlib
import shlex
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
    1. left/right timestamp match
"""


@pytest.mark.rostest
def generate_test_description():
    device_id = 0
    command = f'"if [ -c /dev/video{device_id} ]; then echo Device Found; \
                else echo Device Not Found; fi"'
    result = subprocess.run(shlex.split(command), shell=True, capture_output=True, text=True)

    if(result.stdout.strip() == 'Device Found'):
        IsaacArgusBenchTest.skip_test = False

        argus_stereo_node = ComposableNode(
            name='argus_stereo',
            package='isaac_ros_argus_camera',
            plugin='nvidia::isaac_ros::argus::ArgusStereoNode',
            namespace=IsaacArgusBenchTest.generate_namespace(),
            parameters=[{'camera_id': device_id}]
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
            received_messages = {}

            topics = ['left/image_raw', 'right/image_raw', 'left/camerainfo', 'right/camerainfo']
            self.generate_namespace_lookup(topics)

            subs = self.create_logging_subscribers(
                [('left/image_raw', Image), ('right/image_raw', Image),
                 ('left/camerainfo', CameraInfo), ('right/camerainfo', CameraInfo)],
                received_messages,
                accept_multiple_messages=True)
            try:
                end_time = time.time() + TIMEOUT
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=(0.01))

                # Check all messages are received
                for topic in topics:
                    self.assertTrue(len(received_messages[topic]) > 0, f'{topic} is not received')

                # Record the statistic of all topics
                jitters, timestamps_ms, frame_rates, durations = {}, {}, {}, {}
                exceed_tolerance_counts = defaultdict(int)
                for topic in topics:
                    messages = received_messages[topic]
                    timestamps_ms[topic] = []
                    for message in messages:
                        message_timestamp_ms = \
                            message.header.stamp.sec * SEC_TO_MS + \
                            message.header.stamp.nanosec / MS_TO_NS
                        timestamps_ms[topic].append(message_timestamp_ms)

                    expected_diff = SEC_TO_MS/EXPECTED_FPS
                    jitters[topic] = numpy.abs(numpy.diff(timestamps_ms[topic]))
                    for index in range(len(jitters[topic])):
                        jitter = abs(jitters[topic][index] - expected_diff)
                        if(jitter > JITTER_TOLERANCE):
                            exceed_tolerance_counts[topic] += 1
                        jitters[topic][index] = jitter

                    durations[topic] = timestamps_ms[topic][-1] - timestamps_ms[topic][0]
                    if durations[topic] > 0:
                        frame_rates[topic] = \
                            len(received_messages[topic]) / (durations[topic] / SEC_TO_MS)
                    else:
                        self.node.get_logger().warning(
                            'Could not compute MEAN_FRAME_RATE due to an invalid value of'
                            f'RECEIVED_DURATION = {durations[topic]}')
                        frame_rates[topic] = 0

                # Find matched timestamps
                left_cam_timestamps = timestamps_ms['left/camerainfo']
                right_cam_timestamps = timestamps_ms['right/camerainfo']
                left_index, right_index, matched_timestamps = 0, 0, 0
                max_received_cam_length = max(len(left_cam_timestamps), len(right_cam_timestamps))
                while (left_index < len(left_cam_timestamps) and
                       right_index < len(right_cam_timestamps)):
                    if left_cam_timestamps[left_index] == right_cam_timestamps[right_index]:
                        left_index += 1
                        right_index += 1
                        matched_timestamps += 1
                    elif left_cam_timestamps[left_index] < right_cam_timestamps[right_index]:
                        left_index += 1
                    else:
                        right_index += 1

                # Report message statistics
                for topic in topics:
                    heading = f'Argus Camera Topic {topic} Statistics'
                    self.node.get_logger().info('+-{}-+'.format('-'*MAX_ROW_WIDTH))
                    self.node.get_logger().info(
                        '| {:^{width}} |'.format(heading, width=MAX_ROW_WIDTH))
                    self.node.get_logger().info('+-{}-+'.format('-'*MAX_ROW_WIDTH))
                    self.node.get_logger().info(
                        f'# of Received Frames: {len(received_messages[topic])}')
                    self.node.get_logger().info(
                        f'Delta between First & Last Received Frames (ms): '
                        f'{float(durations[topic])}')
                    self.node.get_logger().info(
                        f'Mean Frame Rate (fps) : {frame_rates[topic]}')
                    self.node.get_logger().info(
                        f'Max. Jitter (ms) : {float(numpy.max(jitters[topic]))}')
                    self.node.get_logger().info(
                        f'Min. Jitter (ms) : {float(numpy.min(jitters[topic]))}')
                    self.node.get_logger().info(
                        f'Mean. Jitter (ms) : {float(numpy.mean(jitters[topic]))}')
                    self.node.get_logger().info(
                        f'% of Jitters out of tolerance ({JITTER_TOLERANCE} ms) : '
                        f'{float(exceed_tolerance_counts[topic] * 100.0 / len(jitters[topic]))}')
                    self.node.get_logger().info('+-{}-+'.format('-'*MAX_ROW_WIDTH))

                # Report timestamp match statistics
                heading = 'Argus Camera Topic Matching Statistics'
                self.node.get_logger().info('+-{}-+'.format('-'*MAX_ROW_WIDTH))
                self.node.get_logger().info(
                    '| {:^{width}} |'.format(heading, width=MAX_ROW_WIDTH))
                self.node.get_logger().info(
                    f'# of Matched Pairs: {matched_timestamps}')
                self.node.get_logger().info(
                    f'% of Matched Pairs : '
                    f'{float(matched_timestamps * 100.0 / max_received_cam_length)}')
                self.node.get_logger().info('+-{}-+'.format('-'*MAX_ROW_WIDTH))

            finally:
                self.node.destroy_subscription(subs)
