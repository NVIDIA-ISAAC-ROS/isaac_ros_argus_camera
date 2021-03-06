# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import pathlib
import time

import cv2
from cv_bridge import CvBridge
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy

from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.rostest
def generate_test_description():
    argus_mono_node = ComposableNode(
        name='argus_mono',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace=IsaacArgusMonoNodeTest.generate_namespace(),
    )

    return IsaacArgusMonoNodeTest.generate_test_description([
        ComposableNodeContainer(
            name='argus_mono_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[argus_mono_node],
            namespace=IsaacArgusMonoNodeTest.generate_namespace(),
            output='screen',
            arguments=['--ros-args', '--log-level', 'info',
                       '--log-level', 'isaac_ros_test.argus_mono:=debug',
                       '--log-level', 'NitrosImage:=debug',
                       '--log-level', 'NitrosCameraInfo:=debug'],
        )
    ])


class IsaacArgusMonoNodeTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_image_capture(self):
        """
        Test image capture from argus mono camera.

        Test that the functionality to read the image message from argus mono camera and save
        it to the disk.
        """
        TIMEOUT = 10
        received_messages = {}

        self.generate_namespace_lookup(['left/image_raw', 'left/camerainfo'])

        subs = self.create_logging_subscribers(
            [('left/image_raw', Image), ('left/camerainfo', CameraInfo)], received_messages,
            accept_multiple_messages=True)
        try:
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:

                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if len(received_messages['left/image_raw']) > 0 and \
                        len(received_messages['left/camerainfo']) > 0:
                    done = True
                    break
            self.assertTrue(done, 'Appropriate output not received')
            left_image = received_messages['left/image_raw']

            left_cv_image = CvBridge().imgmsg_to_cv2(left_image[0], desired_encoding='bgr8')
            cv2.imwrite('left_image.png', left_cv_image)

        finally:
            self.node.destroy_subscription(subs)
