# Isaac ROS Argus Camera

ROS 2 packages based on NVIDIA `libArgus` library for NVIDIA-accelerated CSI camera support.

<div align="center"><img src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_sample_raw.png/" width="300px" title="raw image"/>
<img src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_sample_isp.png/" width="300px" title="isp processed image"/></div>

## Overview

The Isaac ROS Argus Camera module contains an ROS 2 package for sensor
processing to output images. Image sensors are connected on CSI and GMSL
hardware interfaces to Jetson platforms. This package uses dedicated
hardware engines to accelerate image processing. Output images are used
in graphs of nodes for AI and CV perception packages, image compression
for capture to disk by event recorders, and live-stream visuals for
remote robot teleoperation.

Isaac ROS Argus Camera provides with several sensor capture and
processing features, including AWB (auto-white-balance), AE
(auto-exposure), and noise reduction. Leveraging hardware engines in
Jetson, Argus provides multi-camera frame synchronization, with very
high precision frame acquisition timestamping and jitter less than
100us.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera_nodegraph.png/" width="800px"/></a></div>

In the example graph of nodes above, the Argus Camera module processes
sensor image data from the camera for input to vision-based perception
graphs, including DNN stereo disparity, AprilTag, VSLAM, and H.264
encode. Each of the nodes in green is GPU accelerated for a
high-performance compute graph from Argus camera to vision-based
perception functions.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera_zeromemcpy.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera_zeromemcpy.png/" width="800px"/></a></div>

Argus Camera uses dedicated hardware engines to access the full memory
bandwidth in Jetson. Raw camera images are delivered via CSI or GMSL
interfaces directly to the GPU accelerated memory. The ISP hardware
processes the raw image directly into a GPU accelerated output image
topic.

Widely available USB and Ethernet plug-in cameras can be used for
robotics applications, but there is performance cost for this
convenience. The I/O interface (USB or Ethernet) places the image from
the camera directly into CPU-accessible memory. The camera driver makes
a copy from the I/O interface using the CPU to make the image available
to other applications. The Camera driver wrapper node in ROS performs
another memory copy with the CPU from the driver to publish the image in ROS.
Before a USB or Ethernet image arrives as a published topic, two CPU
memory copy actions have been performed for every pixel. In contrast, the
Argus Camera module processes sensor data into output image topics in
ROS without the CPU touching a single pixel in the image.

> [!Note]
> Argus Camera outputs `sensor_msgs/Image` at the sensor data
> rate, subject to performance capabilities of the Jetson platform(s).
> For example, a [Hawk
> camera](https://leopardimaging.com/leopard-imaging-hawk-stereo-camera/)
> configured for 30fps (frames per second) stereo 1920x1080 will output
> time-synchronized left and right camera frames `sensor_msgs/Image` at
> 30fps.

> [!Note]
> Argus Camera is not supported on x86_64 platforms
> with discrete GPUs that do not have a CSI or GMSL interface to
> connect to a camera.

> [!Note]
> See
> [Argus](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html)
> for more information on camera processing.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_argus_camera/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_argus_camera`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_argus_camera/isaac_ros_argus_camera/index.html#api)

## Latest

Update 2024-12-10: Support selecting camera module by index
