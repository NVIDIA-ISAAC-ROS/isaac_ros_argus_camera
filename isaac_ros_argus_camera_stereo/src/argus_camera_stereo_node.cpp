/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_argus_camera_stereo/argus_camera_stereo_node.hpp"

// Ignore warnings in system library headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <Argus/Argus.h>
#include <Argus/EGLSync.h>
#include <ArgusHelpers.h>
#include <Argus/Ext/SyncSensorCalibrationData.h>
#include <EGLStream/EGLStream.h>
#include <EGLGlobal.h>
#include <GLContext.h>
#include <NativeBuffer.cpp>
#include <nvmmapi/NvNativeBuffer.cpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/EGLInterop.h>
#include <vpi/Image.h>
#include <vpi/ImageFormat.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Stream.h>
#include <vpi/VPI.h>
#pragma GCC diagnostic pop

#include <vector>
#include <string>
#include <unordered_map>

#include "cv_bridge/cv_bridge.h"
#include "isaac_ros_common/vpi_utilities.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

// Ignore warnings in system library headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "Thread.cpp"
#pragma GCC diagnostic pop

namespace isaac_ros
{
namespace argus
{

static const uint32_t NUM_BUFFERS = 10;

void SyncStereoCalibrationData(
  const Argus::Ext::ISyncSensorCalibrationData * sync_sensor_calibration_data,
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  Argus::Size2D<uint32_t> image_size = sync_sensor_calibration_data->getImageSizeInPixels();
  Argus::DistortionType lens_distortion_type =
    sync_sensor_calibration_data->getLensDistortionType();

  Argus::Point2D<float> principal_point = sync_sensor_calibration_data->getPrincipalPoint();
  Argus::Point2D<float> focal_length = sync_sensor_calibration_data->getFocalLength();

  uint32_t radial_coeffs_count = sync_sensor_calibration_data->getRadialCoeffsCount(
    lens_distortion_type);
  std::vector<float> dist;
  sync_sensor_calibration_data->getRadialCoeffs(&dist, lens_distortion_type);

  Argus::Point3D<float> rot3d = sync_sensor_calibration_data->getRotationParams();
  Argus::Point3D<float> translation = sync_sensor_calibration_data->getTranslationParams();

  float skew = sync_sensor_calibration_data->getSkew();

  camera_info->width = image_size.width();
  camera_info->height = image_size.height();
  camera_info->distortion_model = static_cast<std::string>(lens_distortion_type.getName());

  if (camera_info->width == 0 || camera_info->height == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "CameraCalibration"),
      "Camera Calibration returned image dimension as 0. Is the camera properly calibrated?");
    return;
  }

  // Filling in intrinsic camera matrix
  camera_info->k[0] = (float64_t)focal_length.x();
  camera_info->k[1] = skew;
  camera_info->k[2] = (float64_t)principal_point.x();
  camera_info->k[3] = 0;
  camera_info->k[4] = (float64_t)focal_length.y();
  camera_info->k[5] = (float64_t)principal_point.y();
  camera_info->k[6] = 0;
  camera_info->k[7] = 0;
  camera_info->k[8] = 1;

  // Compute rotation matrix
  float64_t rx = (float64_t)rot3d.x();
  float64_t ry = (float64_t)rot3d.y();
  float64_t rz = (float64_t)rot3d.z();

  if (rx != 0 || ry != 0 || rz != 0) {
    float64_t angle = sqrt(rx * rx + ry * ry + rz * rz);
    std::vector<float64_t> axis{rx / angle, ry / angle, rz / angle};
    float64_t s = sin(angle), c = cos(angle);
    float64_t * rm = new float64_t[9];

    // Create cross product matrix for axis vector
    rm[0] = 0;
    rm[1] = -axis[2];
    rm[2] = axis[1];
    rm[3] = axis[2];
    rm[4] = 0;
    rm[5] = -axis[0];
    rm[6] = -axis[1];
    rm[7] = axis[0];
    rm[8] = 0;

    // Fill Rotation matrix
    camera_info->r[0] = c + (1 - c) * axis[0] * axis[0] + s * rm[0];
    camera_info->r[1] = (1 - c) * axis[0] * axis[1] + s * rm[1];
    camera_info->r[2] = (1 - c) * axis[0] * axis[2] + s * rm[2];
    camera_info->r[3] = (1 - c) * axis[1] * axis[0] + s * rm[3];
    camera_info->r[4] = c + (1 - c) * axis[1] * axis[1] + s * rm[4];
    camera_info->r[5] = (1 - c) * axis[1] * axis[2] + s * rm[5];
    camera_info->r[6] = (1 - c) * axis[2] * axis[0] + s * rm[6];
    camera_info->r[7] = (1 - c) * axis[2] * axis[1] + s * rm[7];
    camera_info->r[8] = c + (1 - c) * axis[2] * axis[2] + s * rm[8];

    delete[] rm;
  }

  // Projection matrix specifying the camera intrinsics for rectified image
  camera_info->p[0] = camera_info->k[0];
  camera_info->p[1] = camera_info->k[1];
  camera_info->p[2] = camera_info->k[2];
  camera_info->p[3] = translation.x();
  camera_info->p[4] = camera_info->k[3];
  camera_info->p[5] = camera_info->k[4];
  camera_info->p[6] = camera_info->k[5];
  camera_info->p[7] = translation.y();
  camera_info->p[8] = camera_info->k[6];
  camera_info->p[9] = camera_info->k[7];
  camera_info->p[10] = camera_info->k[8];
  camera_info->p[11] = translation.z();

  std::string distortion_type_polynomial = static_cast<std::string>("DISTORTION_TYPE_POLYNOMIAL");
  std::string distortion_type_fisheye = static_cast<std::string>("DISTORTION_TYPE_FISHEYE");

  if (distortion_type_polynomial == lens_distortion_type.getName()) {
    camera_info->distortion_model = static_cast<std::string>("rational_polynomial");

    uint32_t tangential_coeffs_count = sync_sensor_calibration_data->getTangentialCoeffsCount();
    std::vector<float> tang;
    sync_sensor_calibration_data->getTangentialCoeffs(&tang);

    // Filling in distortion parameters in the required format
    if (radial_coeffs_count != 0 || tangential_coeffs_count != 0) {
      camera_info->d.resize(radial_coeffs_count + tangential_coeffs_count);
      camera_info->d[0] = (float64_t)dist[0];
      camera_info->d[1] = (float64_t)dist[1];
      for (uint32_t i = 2; i < 2 + tangential_coeffs_count; i++) {
        camera_info->d[i] = (float64_t)tang[i - 2];
      }
      for (uint32_t i = 2; i < radial_coeffs_count; i++) {
        camera_info->d[i + tangential_coeffs_count] = (float64_t)dist[i];
      }
    }
  } else if (distortion_type_fisheye == lens_distortion_type.getName()) {
    camera_info->distortion_model = static_cast<std::string>("plumb_bob");

    if (radial_coeffs_count != 0) {
      camera_info->d.resize(radial_coeffs_count);
      for (uint32_t i = 0; i < radial_coeffs_count; i++) {
        camera_info->d[i] = (float64_t)dist[i];
      }
    }
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "CameraCalibration"),
      "Distortion Model is not supported by ROS. Currently polynomial and fisheye are supported.");
    return;
  }
}

void ComputeStereoRectificationData(sensor_msgs::msg::CameraInfo::SharedPtr camerainfo[])
{
  std::string distortion_type_polynomial = static_cast<std::string>("DISTORTION_TYPE_POLYNOMIAL");
  std::string distortion_type_fisheye = static_cast<std::string>("DISTORTION_TYPE_FISHEYE");

  cv::Matx33d camera_intrinsics[2];
  cv::Matx<double, 1, 8> polynomial_distortion_parameters[2];
  cv::Matx<double, 1, 4> fisheye_distortion_parameters[2];
  cv::Matx31d translation_parameters;
  cv::Matx33d rectification_matrix[2];
  cv::Matx34d projection_matrix[2];
  cv::Matx44d disparity_to_depth_mapping;
  cv::Matx33d rotation_matrix;

  for (uint32_t i = 0; i < 2; i++) {
    camera_intrinsics[i] = {
      camerainfo[i]->k[0],
      camerainfo[i]->k[1],
      camerainfo[i]->k[2],
      camerainfo[i]->k[3],
      camerainfo[i]->k[4],
      camerainfo[i]->k[5],
      camerainfo[i]->k[6],
      camerainfo[i]->k[7],
      camerainfo[i]->k[8]
    };

    if (camerainfo[i]->distortion_model == distortion_type_polynomial) {
      polynomial_distortion_parameters[i] = {
        camerainfo[i]->d[0],
        camerainfo[i]->d[1],
        camerainfo[i]->d[6],
        camerainfo[i]->d[7],
        camerainfo[i]->d[2],
        camerainfo[i]->d[3],
        camerainfo[i]->d[4],
        camerainfo[i]->d[5]
      };
    } else {
      fisheye_distortion_parameters[i] = {
        camerainfo[i]->d[0],
        camerainfo[i]->d[1],
        camerainfo[i]->d[2],
        camerainfo[i]->d[3]
      };
    }
  }

  rotation_matrix = {
    camerainfo[0]->r[0],
    camerainfo[0]->r[1],
    camerainfo[0]->r[2],
    camerainfo[0]->r[3],
    camerainfo[0]->r[4],
    camerainfo[0]->r[5],
    camerainfo[0]->r[6],
    camerainfo[0]->r[7],
    camerainfo[0]->r[8]
  };

  translation_parameters = {
    camerainfo[0]->p[3],
    camerainfo[0]->p[7],
    camerainfo[0]->p[11]
  };

  cv::stereoRectify(
    camera_intrinsics[0], polynomial_distortion_parameters[0],
    camera_intrinsics[1], polynomial_distortion_parameters[1],
    cv::Size2i(camerainfo[0]->width, camerainfo[0]->height),
    rotation_matrix, translation_parameters, rectification_matrix[0], rectification_matrix[1],
    projection_matrix[0], projection_matrix[1],
    disparity_to_depth_mapping, cv::CALIB_ZERO_DISPARITY);

  for (uint32_t i = 0; i < 2; i++) {
    camerainfo[i]->r = {
      rectification_matrix[i](0, 0),
      rectification_matrix[i](0, 1),
      rectification_matrix[i](0, 2),
      rectification_matrix[i](1, 0),
      rectification_matrix[i](1, 1),
      rectification_matrix[i](1, 2),
      rectification_matrix[i](2, 0),
      rectification_matrix[i](2, 1),
      rectification_matrix[i](2, 2)
    };
    camerainfo[i]->p = {
      projection_matrix[i](0, 0),
      projection_matrix[i](0, 1),
      projection_matrix[i](0, 2),
      projection_matrix[i](0, 3),
      projection_matrix[i](1, 0),
      projection_matrix[i](1, 1),
      projection_matrix[i](1, 2),
      projection_matrix[i](1, 3),
      projection_matrix[i](2, 0),
      projection_matrix[i](2, 1),
      projection_matrix[i](2, 2),
      projection_matrix[i](2, 3)
    };
  }
}

ArgusStereoNode::ArgusStereoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"argus_stereo", options}
{
  class SyncStereoConsumerThread;

  typedef struct
  {
    char moduleName[MAX_MODULE_STRING];
    uint32_t camDevice[MAX_CAM_DEVICE];
    Argus::UniqueObj<Argus::OutputStream> stream[MAX_CAM_DEVICE];
    Argus::UniqueObj<Argus::CaptureSession> capture_session;
    Argus::UniqueObj<Argus::OutputStreamSettings> stream_settings;
    SyncStereoConsumerThread * sync_stereo_consumer;
    uint32_t sensor_count;
    bool initialized;
  } ModuleInfo;

  class SyncStereoConsumerThread : public ArgusSamples::Thread
  {
public:
    explicit SyncStereoConsumerThread(
      ModuleInfo * module_Info, ArgusStereoNode * node)
    : node_{node}
    {
      left_stream = module_Info->stream[0].get();
      if (module_Info->sensor_count > 1) {
        right_stream = module_Info->stream[1].get();
      } else {
        right_stream = NULL;
      }
    }

    ~SyncStereoConsumerThread()
    {
    }

private:
    // Thread methods

    virtual bool threadInitialize()
    {
      left_buffer_stream = Argus::interface_cast<Argus::IBufferOutputStream>(left_stream);
      if (!left_buffer_stream) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to create left buffer stream interface\n");
        return false;
      }

      if (right_stream) {
        right_buffer_stream = Argus::interface_cast<Argus::IBufferOutputStream>(right_stream);
        if (!right_buffer_stream) {
          RCLCPP_ERROR(
            node_->get_logger(), "Failed to create right buffer stream interface\n");
          return false;
        }
      }

      return true;
    }

    virtual bool threadExecute()
    {
      Argus::Status status = Argus::STATUS_OK;
      Argus::Buffer * left_buf, * right_buf;
      EGLImageKHR left_image, right_image;
      Argus::IEGLImageBuffer * egl_image_buffer;

      VPIStream stream;
      CHECK_STATUS(vpiStreamCreate(0, &stream));

      VPIConvertImageFormatParams vpi_convert_params;
      CHECK_STATUS(vpiInitConvertImageFormatParams(&vpi_convert_params));

      vpi_convert_params.policy = VPI_CONVERSION_CAST;

      // Map the encoding desired string to the number of channels needed
      const std::unordered_map<std::string, int32_t> g_str_to_channels({{"mono8", CV_8UC1},
          {"mono16", CV_16UC1},
          {"bgr8", CV_8UC3},
          {"rgb8", CV_8UC3},
          {"bgra8", CV_8UC4},
          {"rgba8", CV_8UC4}});

      VPIImageFormat vpi_image_format;
      if (node_->encoding_ == static_cast<std::string>("mono8")) {
        vpi_image_format = VPI_IMAGE_FORMAT_U8;
      } else if (node_->encoding_ == static_cast<std::string>("rgb8")) {
        vpi_image_format = VPI_IMAGE_FORMAT_RGB8;
      }

      VPIWrapEGLImageParams vpi_egl_params;
      vpi_egl_params.colorSpec = VPI_COLOR_SPEC_BT601_ER;
      VPIImage vpi_image_left_in, vpi_image_left_out, vpi_image_right_in, vpi_image_right_out;

      uint64_t frame_id = 0;

      RCLCPP_INFO(node_->get_logger(), "Consumer Running");
      while (rclcpp::ok()) {
        left_buf = left_buffer_stream->acquireBuffer(Argus::TIMEOUT_INFINITE, &status);
        if (status == Argus::STATUS_END_OF_STREAM) {
          break;
        }
        egl_image_buffer = Argus::interface_cast<Argus::IEGLImageBuffer>(left_buf);
        left_image = egl_image_buffer->getEGLImage();
        status = Argus::STATUS_OK;
        if (right_buffer_stream) {
          right_buf = right_buffer_stream->acquireBuffer(Argus::TIMEOUT_INFINITE, &status);
          if (status == Argus::STATUS_END_OF_STREAM) {
            break;
          }
          egl_image_buffer = Argus::interface_cast<Argus::IEGLImageBuffer>(right_buf);
          right_image = egl_image_buffer->getEGLImage();
        }

        CHECK_STATUS(
          vpiImageCreate(
            node_->width_, node_->height_, vpi_image_format, 0,
            &vpi_image_left_out));

        CHECK_STATUS(
          vpiImageCreate(
            node_->width_, node_->height_, vpi_image_format, 0,
            &vpi_image_right_out));

        CHECK_STATUS(
          vpiImageCreateEGLImageWrapper(
            left_image, &vpi_egl_params, 0,
            &vpi_image_left_in));

        CHECK_STATUS(
          vpiImageCreateEGLImageWrapper(
            right_image, &vpi_egl_params, 0,
            &vpi_image_right_in));

        CHECK_STATUS(
          vpiSubmitConvertImageFormat(
            stream, VPI_BACKEND_CUDA, vpi_image_left_in,
            vpi_image_left_out, &vpi_convert_params));

        CHECK_STATUS(
          vpiSubmitConvertImageFormat(
            stream, VPI_BACKEND_CUDA, vpi_image_right_in,
            vpi_image_right_out, &vpi_convert_params));

        CHECK_STATUS(vpiStreamSync(stream));

        VPIImageData out_data_left, out_data_right;

        CHECK_STATUS(vpiImageLock(vpi_image_left_out, VPI_LOCK_READ, &out_data_left));
        CHECK_STATUS(vpiImageLock(vpi_image_right_out, VPI_LOCK_READ, &out_data_right));

        cv_bridge::CvImage cv_img;

        auto stamp = node_->now();

        cv_img.encoding = node_->encoding_;

        cv::Mat out_mat_left{out_data_left.planes[0].height, out_data_left.planes[0].width,
          g_str_to_channels.at(node_->encoding_), out_data_left.planes[0].data,
          static_cast<size_t>(out_data_left.planes[0].pitchBytes)};
        cv_img.image = out_mat_left;
        cv_img.toImageMsg(*(node_->images_[0]));
        left_buffer_stream->releaseBuffer(left_buf);

        cv::Mat out_mat_right{out_data_right.planes[0].height, out_data_right.planes[0].width,
          g_str_to_channels.at(node_->encoding_), out_data_right.planes[0].data,
          static_cast<size_t>(out_data_right.planes[0].pitchBytes)};
        cv_img.image = out_mat_right;
        cv_img.toImageMsg(*(node_->images_[1]));
        right_buffer_stream->releaseBuffer(right_buf);

        CHECK_STATUS(vpiImageUnlock(vpi_image_right_out));
        CHECK_STATUS(vpiImageUnlock(vpi_image_left_out));
        vpiImageDestroy(vpi_image_left_out);
        vpiImageDestroy(vpi_image_right_out);
        vpiImageDestroy(vpi_image_left_in);
        vpiImageDestroy(vpi_image_right_in);

        node_->images_[0]->header.stamp = node_->images_[1]->header.stamp =
          node_->camerainfo_[0]->header.stamp =
          node_->camerainfo_[1]->header.stamp = stamp;
        node_->images_[0]->header.frame_id = node_->images_[1]->header.frame_id =
          node_->camerainfo_[0]->header.frame_id = node_->camerainfo_[1]->header.frame_id =
          std::to_string(frame_id++);

        node_->publishers_[0].publish(*(node_->images_[0]), *(node_->camerainfo_[0]));
        node_->publishers_[1].publish(*(node_->images_[1]), *(node_->camerainfo_[1]));

        out_mat_left.release();
        out_mat_right.release();
      }

      RCLCPP_INFO(node_->get_logger(), "Consumer shutting down.");
      requestShutdown();
      return true;
    }

    virtual bool threadShutdown()
    {
      return true;
    }

    Argus::OutputStream * left_stream;
    Argus::OutputStream * right_stream;

    Argus::IBufferOutputStream * left_buffer_stream;
    Argus::IBufferOutputStream * right_buffer_stream;

    ArgusStereoNode * node_;
  };

  auto module_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  module_descriptor.description = "Index specifying the stereo camera module";
  module_descriptor.read_only = false;
  this->declare_parameter("module", 0, module_descriptor);

  auto sensor_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  sensor_descriptor.description = "Index specifying sensor mode for the stereo pair";
  sensor_descriptor.read_only = false;
  this->declare_parameter("sensor", 0, sensor_descriptor);

  auto output_encoding_description = rcl_interfaces::msg::ParameterDescriptor{};
  output_encoding_description.description = "ROS image encoding to use for the output image";
  output_encoding_description.additional_constraints =
    "Currently supported: 'rgb8' or 'mono8' (Default: 'mono8')";
  this->declare_parameter("output_encoding", std::string{"mono8"}, output_encoding_description);

  //
  // Set up Argus API Framework, identify available camera modules and sensor modes, and create
  // a capture session for the selected devices and corresponding sensor modes
  //

  uint32_t module_index, sensor_counter, sensor_index;

  Argus::UniqueObj<Argus::CameraProvider> cameraProvider(Argus::CameraProvider::create());
  Argus::ICameraProvider * i_camera_provider = Argus::interface_cast<Argus::ICameraProvider>(
    cameraProvider);
  if (!i_camera_provider) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get core camera provider interface\n");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Argus Version: %s\n", i_camera_provider->getVersion().c_str());

  // Get camera devices vector and display available camera device models
  std::vector<Argus::CameraDevice *> camera_devices;
  Argus::Status status = i_camera_provider->getCameraDevices(&camera_devices);
  if (camera_devices.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera devices\n");
    return;
  }

  if (camera_devices.size() < 2) {
    RCLCPP_ERROR(this->get_logger(), "Must have at least two devices available\n");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Set the stereo camera module using the node parameter \"module\"");
  RCLCPP_INFO(this->get_logger(), "Set the sensor mode using the node parameter \"sensor\"");
  RCLCPP_INFO(
    this->get_logger(),
    "Set the output image format using the node parameter \"output_encoding\". "
    "Supported encodings: \"mono8\" and \"rgb8\" (Default: \"mono8\")\n");

  ModuleInfo module_info[MAX_CAM_DEVICE];
  uint32_t module_count = 0;
  memset(&module_info, 0, MAX_CAM_DEVICE * sizeof(ModuleInfo));

  // cppcheck-suppress syntaxError
  for (uint32_t i = 0; i < MAX_CAM_DEVICE; i++) {
    module_info[i].initialized = false;
  }

  char syncSensorId[MAX_MODULE_STRING];

  // cppcheck-suppress syntaxError
  for (uint32_t i = 0; i < camera_devices.size(); i++) {
    Argus::ICameraProperties * camera_properties = Argus::interface_cast<Argus::ICameraProperties>(
      camera_devices[i]);
    if (!camera_properties) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get ICameraProperties interface\n");
      return;
    }

    const Argus::Ext::ISyncSensorCalibrationData * sync_sensor_calibration_data =
      Argus::interface_cast<const Argus::Ext::ISyncSensorCalibrationData>(camera_devices[i]);
    if (sync_sensor_calibration_data) {
      sync_sensor_calibration_data->getSyncSensorModuleId(syncSensorId, sizeof(syncSensorId));

      for (uint32_t j = 0; j <= module_count; j++) {
        if (strcmp(syncSensorId, module_info[j].moduleName)) {
          if (module_info[j].initialized == false) {
            snprintf(
              module_info[j].moduleName, sizeof(module_info[j].moduleName), "%s",
              syncSensorId);
            module_info[j].initialized = true;
            module_info[j].camDevice[module_info[j].sensor_count++] = i;
          } else {
            continue;
          }

          module_count++;
          break;
        } else {
          module_info[j].camDevice[module_info[j].sensor_count++] = i;
          break;
        }
      }
    } else {
      std::string model_string;
      model_string = camera_properties->getModuleString();
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to get ISyncSensorCalibrationData interface [1]\n");

      for (uint32_t j = 0; j <= module_count; j++) {
        if (strcmp(model_string.c_str(), module_info[j].moduleName)) {
          if (module_info[j].initialized == false) {
            snprintf(
              module_info[j].moduleName, sizeof(module_info[j].moduleName), "%s",
              model_string.c_str());
            module_info[j].initialized = true;
            module_info[j].camDevice[module_info[j].sensor_count++] = i;
          } else {
            continue;
          }

          module_count++;
          break;
        } else {
          module_info[j].camDevice[module_info[j].sensor_count++] = i;
          break;
        }
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Following camera modules are available:");
  Argus::CameraDevice * camera_device;
  Argus::ICameraProperties * properties;
  Argus::Size2D<uint32_t> res;
  std::vector<Argus::SensorMode *> sensor_modes;
  Argus::ISensorMode * sensor_properties;

  // Assuming both sensors of a stereo camera have same sensor modes
  for (uint32_t i = 0; i < module_count; i++) {
    RCLCPP_INFO(
      this->get_logger(), "Module %d: %s module with %d sensors connected", i,
      module_info[i].moduleName, module_info[i].sensor_count);
    RCLCPP_INFO(this->get_logger(), "Available sensor modes for this module:");

    camera_device = camera_devices[module_info[i].camDevice[0]];
    properties = Argus::interface_cast<Argus::ICameraProperties>(camera_device);
    status = properties->getAllSensorModes(&sensor_modes);
    if (status != Argus::STATUS_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get sensor modes vector\n");
      return;
    }
    sensor_counter = 0;
    for (auto snsr : sensor_modes) {
      sensor_properties = Argus::interface_cast<Argus::ISensorMode>(snsr);
      res = sensor_properties->getResolution();
      RCLCPP_INFO(this->get_logger(), "%d (%d x %d)", sensor_counter++, res.width(), res.height());
    }
  }

  Argus::UniqueObj<Argus::Request> request;
  std::vector<Argus::CameraDevice *> lr_cameras;

  // Get the parameter value for stereo camera module
  this->get_parameter("module", module_index);
  for (uint32_t j = 0; j < module_info[module_index].sensor_count; j++) {
    lr_cameras.push_back(camera_devices[module_info[module_index].camDevice[j]]);
  }

  module_info[module_index].capture_session = Argus::UniqueObj<Argus::CaptureSession>(
    i_camera_provider->createCaptureSession(
      lr_cameras));
  if (!module_info[module_index].capture_session) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create capture session\n");
    return;
  }
  Argus::ICaptureSession * capture_session = Argus::interface_cast<Argus::ICaptureSession>(
    module_info[module_index].capture_session);
  if (!capture_session) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get ICaptureSession interface\n");
    return;
  }

  module_info[module_index].stream_settings = Argus::UniqueObj<Argus::OutputStreamSettings>(
    capture_session->createOutputStreamSettings(
      Argus::STREAM_TYPE_BUFFER));

  Argus::IOutputStreamSettings * stream_settings =
    Argus::interface_cast<Argus::IOutputStreamSettings>(module_info[module_index].stream_settings);
  if (!stream_settings) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get IOutputStreamSettings interface\n");
    return;
  }

  Argus::IBufferOutputStreamSettings * i_buffer_stream_settings =
    Argus::interface_cast<Argus::IBufferOutputStreamSettings>(
    module_info[module_index].stream_settings);
  if (!i_buffer_stream_settings) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get IBufferOutputStreamSettings interface\n");
    return;
  }

  i_buffer_stream_settings->setBufferType(Argus::BUFFER_TYPE_EGL_IMAGE);
  if (i_buffer_stream_settings->setSyncType(Argus::SYNC_TYPE_NONE) != Argus::STATUS_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to disable EGL sync");
    return;
  }

  EGLint major, minor;
  EGLDisplay egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  eglInitialize(egl_display, &major, &minor);

  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    stream_settings->setCameraDevice(lr_cameras[a]);
    module_info[module_index].stream[a] = Argus::UniqueObj<Argus::OutputStream>(
      capture_session->createOutputStream(
        module_info[module_index].stream_settings.get()));
  }

  // Streams for the capture session have been created

  // Get the parameter value for sensor model
  this->get_parameter("sensor", sensor_index);
  Argus::SensorMode * sensor_mode = sensor_modes[sensor_index];
  Argus::ISensorMode * iSensorMode = Argus::interface_cast<Argus::ISensorMode>(sensor_mode);
  if (!iSensorMode) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get sensor mode interface\n");
    return;
  }

  width_ = (iSensorMode->getResolution()).width();
  height_ = (iSensorMode->getResolution()).height();

  Argus::Size2D<uint32_t> size =
    Argus::Size2D<uint32_t>(
    width_,
    height_);

  ArgusSamples::NativeBuffer * native_buffers[MAX_CAM_DEVICE][NUM_BUFFERS];
  EGLImageKHR egl_images[MAX_CAM_DEVICE][NUM_BUFFERS];
  Argus::UniqueObj<Argus::Buffer> buffers[MAX_CAM_DEVICE][NUM_BUFFERS];

  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    Argus::IBufferOutputStream * i_buffer_output_stream =
      Argus::interface_cast<Argus::IBufferOutputStream>(module_info[module_index].stream[a]);
    if (!i_buffer_output_stream) {
      RCLCPP_INFO(this->get_logger(), "Failed to create IBufferOutputStream interface\n");
    }

    Argus::UniqueObj<Argus::BufferSettings> buffer_settings(
      i_buffer_output_stream->createBufferSettings());
    if (!buffer_settings) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create buffer settings\n");
      return;
    }

    Argus::IEGLImageBufferSettings * i_buffer_settings =
      Argus::interface_cast<Argus::IEGLImageBufferSettings>(buffer_settings);
    if (!i_buffer_settings) {
      RCLCPP_ERROR(this->get_logger(), "Cannot get IEGLImageBufferSettings interface\n");
      return;
    }

    i_buffer_settings->setEGLDisplay(egl_display);

    ArgusSamples::NvNativeBuffer * buf;
    for (uint32_t j = 0; j < NUM_BUFFERS; j++) {
      buf = ArgusSamples::NvNativeBuffer::create(size, NvBufferColorFormat_NV12);
      native_buffers[a][j] = (ArgusSamples::NativeBuffer *)buf;
      if (!native_buffers[a][j]) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create native buffer\n");
        return;
      }

      egl_images[a][j] = native_buffers[a][j]->createEGLImage(egl_display);
      if (egl_images[a][j] == EGL_NO_IMAGE_KHR) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create eglImage from native buffer\n");
        return;
      }

      i_buffer_settings->setEGLImage(egl_images[a][j]);

      buffers[a][j].reset(i_buffer_output_stream->createBuffer(buffer_settings.get()));

      if (!Argus::interface_cast<Argus::IEGLImageBuffer>(buffers[a][j])) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create buffer\n");
        return;
      }
      if (i_buffer_output_stream->releaseBuffer(buffers[a][j].get()) != Argus::STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to release buffer for capture use\n");
        return;
      }
    }
  }

  this->get_parameter("output_encoding", encoding_);

  module_info[module_index].sync_stereo_consumer = new SyncStereoConsumerThread(
    &module_info[module_index], this);

  if (!(module_info[module_index].sync_stereo_consumer->initialize())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Consumer Thread\n");
    return;
  }
  if (!(module_info[module_index].sync_stereo_consumer->waitRunning())) {
    RCLCPP_ERROR(this->get_logger(), "Consumer Thread does not enter running state\n");
    return;
  }

  request = Argus::UniqueObj<Argus::Request>(capture_session->createRequest());
  Argus::IRequest * i_request = Argus::interface_cast<Argus::IRequest>(request);
  if (!i_request) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get i_request interface\n");
    return;
  }

  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    i_request->enableOutputStream(module_info[module_index].stream[a].get());
  }

  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    const Argus::Ext::ISyncSensorCalibrationData * sync_sensor_calibration_data =
      Argus::interface_cast<const Argus::Ext::ISyncSensorCalibrationData>(lr_cameras[a]);
    camerainfo_[a] = sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo());
    images_[a] = sensor_msgs::msg::Image::SharedPtr(new sensor_msgs::msg::Image());
    if (sync_sensor_calibration_data) {
      SyncStereoCalibrationData(sync_sensor_calibration_data, camerainfo_[a]);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to get ISyncSensorCalibrationData interface\n");
    }
  }

  ComputeStereoRectificationData(camerainfo_);

  publishers_[0] = image_transport::create_camera_publisher(this, "/stereo/left/image_raw");
  publishers_[1] = image_transport::create_camera_publisher(this, "/stereo/right/image_raw");

  Argus::ISourceSettings * source_settings = Argus::interface_cast<Argus::ISourceSettings>(request);
  if (!source_settings) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get source settings request interface\n");
    return;
  }

  source_settings->setSensorMode(sensor_mode);

  // Submit capture for the specified time.
  RCLCPP_INFO(this->get_logger(), "Starting repeat capture requests.");
  if (capture_session->repeat(request.get()) != Argus::STATUS_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start repeat capture request for preview");
    return;
  }
  while (rclcpp::ok()) {
  }

  capture_session->stopRepeat();
  capture_session->waitForIdle();

  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    module_info[module_index].stream[a].reset();
  }

  RCLCPP_INFO(this->get_logger(), "Producer shutting down");
  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    module_info[module_index].stream[a].reset();
  }
  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    for (uint32_t j = 0; j < NUM_BUFFERS; j++) {
      buffers[a][j].reset();
    }
  }
  for (uint32_t a = 0; a < module_info[module_index].sensor_count; a++) {
    for (uint32_t j = 0; j < NUM_BUFFERS; j++) {
      delete native_buffers[a][j];
    }
  }
  module_info[module_index].capture_session.reset();
  module_info[module_index].stream_settings.reset();
  request.reset();
  cameraProvider.reset();
}

ArgusStereoNode::~ArgusStereoNode()
{
}

}  // namespace argus
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros::argus::ArgusStereoNode)
