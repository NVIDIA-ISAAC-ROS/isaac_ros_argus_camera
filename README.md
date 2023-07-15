# Isaac ROS Argus Camera

<div align="center"><img src="resources/isaac_ros_argus_sample_raw.png" width="300" title="raw image"/> <img src="resources/isaac_ros_argus_sample_isp.png" width="300" title="isp processed image"/></div>

## Overview

The Isaac ROS Argus Camera module contains an ROS 2 package for sensor processing to output images. Image sensors are connected on CSI and GMSL hardware interfaces to Jetson platforms. This package uses dedicated hardware engines to accelerate image processing. Output images are used in graphs of nodes for AI and CV perception packages, image compression for capture to disk by event recorders, and live-stream visuals for remote robot teleoperation.

Isaac ROS Argus Camera provides with several sensor capture and processing features, including AWB (auto-white-balance), AE (auto-exposure), and noise reduction. Leveraging hardware engines in Jetson, Argus provides multi-camera frame synchronization, with very high precision frame acquisition timestamping and jitter less than 100us.

<div align="center"><img src="resources/isaac_ros_argus_camera_nodegraph.png" width="800" title="graph of nodes with Argus Camera"/></div>

In the example graph of nodes above, the Argus Camera module processes sensor image data from the camera for input to vision-based perception graphs, including DNN stereo disparity, AprilTag, VSLAM, and H.264 encode. Each of the nodes in green is GPU accelerated for a high-performance compute graph from Argus camera to vision-based perception functions.

<div align="center"><img src="resources/isaac_ros_argus_camera_zeromemcpy.png" width="800" title="zero CPU memcpy with Argus Camera"/></div>

Argus Camera uses dedicated hardware engines to access the full memory bandwidth in Jetson. Raw camera images are delivered via CSI or GMSL interfaces directly to the GPU accelerated memory. The ISP hardware processes the raw image directly into a GPU accelerated output image topic.

Widely available USB and Ethernet plug-in cameras can be used for robotics applications, but there is performance cost for this convenience. The I/O interface (USB or Ethernet) places the image from the camera directly into CPU-accessible memory. The camera driver makes a copy from the I/O interface using the CPU to make the image available to other applications. The Camera driver wrapper node in ROS performs another memcpy with the CPU from the driver to publish the image in ROS. Before a USB or Ethernet image arrives as a published topic, two CPU memcpy actions have been perfromed for every pixel. In contrast, the Argus Camera module processes sensor data into output image topics in ROS without the CPU touching a single pixel in the image.

> **Note**: Argus Camera outputs sensor_msgs/Image at the sensor data rate, subject to performance capabilites of the Jetson platform(s). For example, a [Hawk camera](https://www.leopardimaging.com/li-ar0234cs-stereo-gmsl2-hawk/) configured for 30fps (frames per second) stereo 1920x1080 will output time-synchronized left and right camera frames sensor_msgs/Image at 30fps.
<!-- Split blockquote -->
> **Note**: Argus Camera is not supported on x86_64 platforms with discrete GPUs that do not have a CSI or GMSL interface to connect to a camera.
<!-- Split blockquote -->
> **Note**: See [Argus](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html) for more information on camera processing.

### Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Table of Contents

- [Isaac ROS Argus Camera](#isaac-ros-argus-camera)
  - [Overview](#overview)
    - [Isaac ROS NITROS Acceleration](#isaac-ros-nitros-acceleration)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
    - [Reference Cameras](#reference-cameras)
  - [Quickstart](#quickstart)
  - [Package Reference](#package-reference)
    - [Usage](#usage)
      - [Launch monocular camera](#launch-monocular-camera)
      - [Launch stereo camera](#launch-stereo-camera)
    - [ROS Parameters](#ros-parameters)
    - [ROS Topics Published](#ros-topics-published)
    - [Launch testing](#launch-testing)
      - [Monocular camera](#monocular-camera)
      - [Stereo camera](#stereo-camera)
    - [Output Color Space Format](#output-color-space-format)
    - [`CameraInfo` Message](#camerainfo-message)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
    - [Argus fails to create capture session](#argus-fails-to-create-capture-session)
      - [Symptoms](#symptoms)
      - [Solution](#solution)
  - [Updates](#updates)

## Latest Update

Update 2023-04-05: Update to be compatible with JetPack 5.1.1

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) with off-the-shelf cameras from NVIDIA partners (see the [Reference Cameras](#reference-cameras) section for more details).
> **Note**: x86_64 system is not supported.
<!-- Split blockquote -->
> **Note**: Versions of ROS 2 earlier than Humble are **not** supported. This package depends on specific ROS 2 implementation features that were only introduced beginning with the Humble release.

| Platform | Hardware                                                                                                                                                                                                  | Software                                                       | Notes                                                                                                                                                                                                                                                                                                                                                         |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) </br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack) | The [CSI](https://en.wikipedia.org/wiki/Camera_Serial_Interface) camera device needs to be connected and presented as a video device (e.g. `/dev/video0`). </br></br> For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on Jetson platforms.

> **Note**: All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.
  
### Reference Cameras

NVIDIA has worked with our camera partners to provide the modules listed below which are compatible with the Isaac ROS Argus Camera package.

| Camera Type            | Connector | Resolution/FrameRate | H/V FOV | Shutter | Supplier, HW Part, Link to purchase                                                                                                                                   |
| ---------------------- | --------- | -------------------- | ------- | ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Stereo Camera/Color    | GMSL      | 1920x1200@60fps      | 121/73  | Global  | [LI-AR0234CS-STEREO-GMSL2](https://www.leopardimaging.com/product/autonomous-camera/maxim-gmsl2-cameras/li-ar0234cs-stereo-gmsl2/li-ar0234cs-stereo-gmsl2/)           |
| Fisheye Camera/Color   | GMSL      | 1920x1200@60fps      | 202/127 | Global  | [LI-AR0234CS-GMSL2-OWL](https://www.leopardimaging.com/product/autonomous-camera/maxim-gmsl2-cameras/li-ar0234cs-gmsl2-owl/li-ar0234cs-gmsl2-owl/)                    |
| Monocular Camera/Color | CSI       | 4056x3040@60fps      | 140/103 | Global  | [IMX477 -140FOH](https://www.leopardimaging.com/product/csi-2-mipi-modules-i-pex/csi-2-mipi-modules/rolling-shutter-mipi-cameras/12-33mp-imx477/li-imx477-mipi-140h/) |
| Monocular Camera/Color | CSI       | 3264x2464@21fps      | 90/90   | Rolling | [LI-NANO-CB-IMX219-090H](https://www.leopardimaging.com/product/nvidia-jetson-cameras/nvidia_nano_mipi_camera_kits/li-nano-cb-imx219-x/li-nano-cb-imx219-090h/)       |

## Quickstart

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).  

2. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera
    ```

3. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev && \
      ./src/isaac_ros_common/scripts/run_dev.sh
    ```

4. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

5. (Optional) Run tests to verify complete and correct installation:

    ```bash
    colcon test --executor sequential
    ```

6. Run the following launch files to spin up a demo of this package:

    ```bash
    ros2 launch isaac_ros_argus_camera isaac_ros_argus_camera_mono.launch.py
    ```

7. Use `image_saver` to save the output images:

    ```bash
    ros2 run image_view image_saver --ros-args -r image:=/left/image_raw -p filename_format:="left_image.jpg"
    ```

## Package Reference

### Usage

#### Launch monocular camera

```bash
ros2 launch isaac_ros_argus_camera isaac_ros_argus_camera_mono.launch.py
```

#### Launch stereo camera

```bash
ros2 launch isaac_ros_argus_camera isaac_ros_argus_camera_stereo.launch.py
```

### ROS Parameters

| ROS Parameter              | Type     | Default     | Description                                                                                                     |
| -------------------------- | -------- | ----------- | --------------------------------------------------------------------------------------------------------------- |
| `camera_id`                | `uint`   | `0`         | The video device index </br> E.g. `/dev/video0`                                                                 |
| `module_id`                | `uint`   | `0`         | The camera module index in the device tree when there is more than one of the same camera module connected      |
| `mode`                     | `uint`   | `0`         | The resolution mode supported by the camera sensor and driver.                                                  |
| `camera_type`              | `uint`   | `1`         | 0 for Monocular type camera; 1 for Stereo type camera                                                           |
| `camera_info_url`          | `string` | N/A         | Optional url of a camera info `.ini` file for monocular camera to read intrinsic information                    |
| `left_camera_info_url`     | `string` | N/A         | Optional url of a camera info `.ini` file for the left imager of a stereo camera to read intrinsic information  |
| `right_camera_info_url`    | `string` | N/A         | Optional url of a camera info `.ini` file for the right imager of a stereo camera to read intrinsic information |
| `camera_link_frame_name`   | `string` | `camera`    | The frame name associated with the origin of the camera body.                                                   |
| `optical_frame_name`       | `string` | `left_cam`  | The frame name associated with the imager inside camera body (for monocular camera).                            |
| `left_optical_frame_name`  | `string` | `left_cam`  | The frame name associated with the left imager inside camera body (for stereo camera).                          |
| `right_optical_frame_name` | `string` | `right_cam` | The frame name associated with the right imager inside camera body (for stereo camera).                         |

> **Note**: To run the stereo camera, two video devices should present for the left and right sensors, respectively (e.g. `/dev/video0` and `/dev/video1`).

### ROS Topics Published

| ROS Topic          | Interface                                                                                                      | Description                       |
| ------------------ | -------------------------------------------------------------------------------------------------------------- | --------------------------------- |
| `left/image_raw`   | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)           | The left image of a stereo pair.  |
| `right/image_raw`  | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)           | The right image of a stereo pair. |
| `left/camerainfo`  | [sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) | The left camera model.            |
| `right/camerainfo` | [sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) | The right camera model.           |

### Launch testing

#### Monocular camera

   ```bash
launch_test src/isaac_ros_argus_camera/isaac_ros_argus_camera/test/isaac_ros_argus_camera_mono_test.py
   ```

#### Stereo camera

   ```bash
launch_test src/isaac_ros_argus_camera/isaac_ros_argus_camera/test/isaac_ros_argus_camera_stereo_test.py
   ```

### Output Color Space Format

The Isaac ROS Argus node supports the `YUV444` and `YUV420` colorspaces from `libargus` and converts it to the `RGB888` colorspace as output.

### `CameraInfo` Message

The Isaac ROS Argus node uses the Argus Ext API to retrieve calibration parameters from the camera through the Linux device driver and convert it into [CameraInfo](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) messages.
Refer to [this page](https://docs.nvidia.com/jetson/l4t-multimedia/classArgus_1_1Ext_1_1ISyncSensorCalibrationData.html) for the data structure of the calibration parameters.

> **Note**: Each camera module should have stored the calibration parameters in internal memory like EEPROM, and the device driver must support the Argus Ext API to extract those parameters. Contact your camera vendor to get the required drivers.
<!-- Split blockquote -->
> **Note**: If your camera does not support the Argus Ext API, you can also specify a URL to a camera info `.ini` file parseable by the ROS [CameraInfoManager](http://wiki.ros.org/camera_info_manager) using the `camera_info_url` parameter on the Isaac ROS Argus node. This will allow you to provide parameters you may have calibrated using the ROS Camera Calibration package, for example.
<!-- Split blockquote -->
> **Note**: When the `camera_info_url` is provided, the loaded parameters override the `CameraInfo` from Argus Ext API.

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

### Argus fails to create capture session

#### Symptoms

The Isaac ROS Argus node can fail to create a capture session inside the container after the `nvargus` daemon has crashed. By default, the `nvargus` daemon is running in background, but it may crash due to other Argus clients. This will prevent Argus camera nodes from creating capture sessions. You may see messages with errors similar to `Failed to create capture session`.

#### Solution

Exit the Docker container and restart the `nvargus` daemon by running `sudo systemctl restart nvargus-daemon.service`

## Updates

| Date       | Changes                                    |
| ---------- | ------------------------------------------ |
| 2023-04-05 | Update to be compatible with JetPack 5.1.1 |
| 2022-10-19 | Updated OSS licensing                      |
| 2022-08-31 | Update to be compatible with JetPack 5.0.2 |
| 2022-06-30 | Support NITROS acceleration                |
| 2022-03-18 | Support CameraInfo from URL                |
| 2021-10-20 | Initial release                            |
