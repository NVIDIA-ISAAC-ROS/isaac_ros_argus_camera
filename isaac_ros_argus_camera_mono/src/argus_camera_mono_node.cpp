/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#include "isaac_ros_argus_camera_mono/argus_camera_mono_node.hpp"

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

// Argus Ext API to retrieve calibration parameters from driver
void getCalibrationData(
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

  float skew = sync_sensor_calibration_data->getSkew();

  camera_info->width = image_size.width();
  camera_info->height = image_size.height();
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

  // For monocular camera, rotation matrix is identity matrix
  camera_info->r[0] = 1;
  camera_info->r[1] = 0;
  camera_info->r[2] = 0;
  camera_info->r[3] = 0;
  camera_info->r[4] = 1;
  camera_info->r[5] = 0;
  camera_info->r[6] = 0;
  camera_info->r[7] = 0;
  camera_info->r[8] = 1;

  // Projection matrix specifying the camera intrinsics for rectified image
  camera_info->p[0] = camera_info->k[0];
  camera_info->p[1] = camera_info->k[1];
  camera_info->p[2] = camera_info->k[2];
  camera_info->p[3] = 0;
  camera_info->p[4] = camera_info->k[3];
  camera_info->p[5] = camera_info->k[4];
  camera_info->p[6] = camera_info->k[5];
  camera_info->p[7] = 0;
  camera_info->p[8] = camera_info->k[6];
  camera_info->p[9] = camera_info->k[7];
  camera_info->p[10] = camera_info->k[8];
  camera_info->p[11] = 0;

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

MonocularNode::MonocularNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"isaac_ros_argus_camera_mono", options}
{
  // Consumer Thread Class
  class MonocularConsumerThread : public ArgusSamples::Thread
  {
public:
    explicit MonocularConsumerThread(MonocularNode * node, Argus::OutputStream * stream)
    : node_{node}, stream_{stream}
    {
    }

    ~MonocularConsumerThread()
    {
    }

private:
    // Thread methods
    virtual bool threadInitialize()
    {
      buffer_stream = Argus::interface_cast<Argus::IBufferOutputStream>(stream_);
      if (!buffer_stream) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to create buffer stream interface in consumer thread\n");
        return false;
      }

      return true;
    }

    virtual bool threadExecute()
    {
      VPIStream stream;
      CHECK_STATUS(vpiStreamCreate(0, &stream));

      uint64_t frame_id = 0;

      RCLCPP_INFO(node_->get_logger(), "Consumer Running");
      while (rclcpp::ok()) {
        Argus::Status status = Argus::STATUS_OK;
        Argus::Buffer * buf_ptr;
        buf_ptr = buffer_stream->acquireBuffer(Argus::TIMEOUT_INFINITE, &status);

        if (status == Argus::STATUS_END_OF_STREAM) {
          return true;
        }

        Argus::IEGLImageBuffer * egl_image_buffer = Argus::interface_cast<Argus::IEGLImageBuffer>(
          buf_ptr);
        if (!egl_image_buffer) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to create IEGLImageBuffer interface\n");
          return false;
        }

        EGLImageKHR egl_image = egl_image_buffer->getEGLImage();

        // Map the encoding desired string to the number of channels needed
        const std::unordered_map<std::string, int> g_str_to_channels({
              {"mono8", CV_8UC1},
              {"mono16", CV_16UC1},
              {"bgr8", CV_8UC3},
              {"rgb8", CV_8UC3},
              {"bgra8", CV_8UC4},
              {"rgba8", CV_8UC4}});

        std::string encoding;
        node_->get_parameter("output_encoding", encoding);
        VPIImageFormat vpi_image_format;
        if (encoding == static_cast<std::string>("mono8")) {
          vpi_image_format = VPI_IMAGE_FORMAT_U8;
        } else if (encoding == static_cast<std::string>("rgb8")) {
          vpi_image_format = VPI_IMAGE_FORMAT_RGB8;
        }

        VPIWrapEGLImageParams vpi_egl_params;
        vpi_egl_params.colorSpec = VPI_COLOR_SPEC_BT601_ER;
        VPIImage vpi_image_in, vpi_image_out;

        CHECK_STATUS(
          vpiImageCreate(
            node_->width_, node_->height_,
            vpi_image_format, 0, &vpi_image_out));

        CHECK_STATUS(vpiImageCreateEGLImageWrapper(egl_image, &vpi_egl_params, 0, &vpi_image_in));

        VPIConvertImageFormatParams vpi_convert_params;
        CHECK_STATUS(vpiInitConvertImageFormatParams(&vpi_convert_params));

        vpi_convert_params.policy = VPI_CONVERSION_CAST;

        CHECK_STATUS(
          vpiSubmitConvertImageFormat(
            stream, VPI_BACKEND_CUDA, vpi_image_in, vpi_image_out,
            &vpi_convert_params));

        CHECK_STATUS(vpiStreamSync(stream));

        VPIImageData out_data;
        vpiImageLock(vpi_image_out, VPI_LOCK_READ, &out_data);
        cv::Mat out_mat{out_data.planes[0].height, out_data.planes[0].width,
          g_str_to_channels.at(encoding), out_data.planes[0].data,
          static_cast<size_t>(out_data.planes[0].pitchBytes)};

        buffer_stream->releaseBuffer(buf_ptr);

        cv_bridge::CvImage cv_img;
        cv_img.image = out_mat;
        cv_img.encoding = encoding;
        cv_img.toImageMsg(*(node_->image_));

        auto stamp = node_->now();
        node_->camerainfo_->header.stamp = node_->image_->header.stamp = stamp;
        node_->camerainfo_->header.frame_id = node_->image_->header.frame_id = std::to_string(
          frame_id++);

        if (node_->get_node_options().use_intra_process_comms()) {
          RCLCPP_DEBUG_STREAM(
            node_->get_logger(),
            "Image message address [PUBLISH]:\t" << node_->image_.get());
          node_->image_pub_->publish(*(node_->image_));
          node_->camera_pub_->publish(*(node_->camerainfo_));
        } else {
          node_->publisher_.publish(*(node_->image_), *(node_->camerainfo_));
        }

        vpiImageDestroy(vpi_image_in);
        CHECK_STATUS(vpiImageUnlock(vpi_image_out));
        vpiImageDestroy(vpi_image_out);

        out_mat.release();
      }

      vpiStreamDestroy(stream);
      RCLCPP_INFO(node_->get_logger(), "Consumer shutting down.");
      requestShutdown();
      return true;
    }

    virtual bool threadShutdown() {return true;}

    MonocularNode * node_;
    Argus::OutputStream * stream_;
    Argus::IBufferOutputStream * buffer_stream;
  };

  if (options.use_intra_process_comms()) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_raw", 20);
    camera_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 20);
  } else {
    publisher_ = image_transport::create_camera_publisher(this, "/image_raw");
  }

  auto device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  device_descriptor.description = "Index specifying camera device model";
  device_descriptor.read_only = false;
  this->declare_parameter("device", 0, device_descriptor);

  auto sensor_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  sensor_descriptor.description = "Index specifying sensor mode";
  sensor_descriptor.read_only = false;
  this->declare_parameter("sensor", 0, sensor_descriptor);

  auto output_encoding_description = rcl_interfaces::msg::ParameterDescriptor{};
  output_encoding_description.description = "ROS image encoding to use for the output image";
  output_encoding_description.additional_constraints =
    "Currently supported: 'rgb8' or 'mono8' (Default: 'mono8')";
  this->declare_parameter("output_encoding", std::string{"mono8"}, output_encoding_description);

  // Set up Argus API Framework, identify available camera devices, and create a capture session for
  // the selected device and sensor mode
  uint32_t device_index, sensor_index, dvc_idx = 0, snsr_idx;

  Argus::UniqueObj<Argus::CameraProvider> cameraProvider(Argus::CameraProvider::create());
  Argus::ICameraProvider * camera_provider = Argus::interface_cast<Argus::ICameraProvider>(
    cameraProvider);
  if (!camera_provider) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get core camera provider interface\n");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Argus Version: %s\n", camera_provider->getVersion().c_str());

  // Get camera devices vector and display available camera device models
  std::vector<Argus::CameraDevice *> camera_devices;
  Argus::Status status = camera_provider->getCameraDevices(&camera_devices);
  if (camera_devices.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera devices");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Set the camera model using the node parameter \"device\"");
  RCLCPP_INFO(this->get_logger(), "Set the sensor mode using the node parameter \"sensor\"");
  RCLCPP_INFO(
    this->get_logger(),
    "Set the output image format using the node parameter \"output_encoding\". "
    "Supported encodings: \"mono8\" and \"rgb8\"\n");
  RCLCPP_INFO(this->get_logger(), "Following camera model indices are available:");

  Argus::ICameraProperties * properties;
  std::string model_string;
  std::vector<Argus::SensorMode *> sensor_modes;
  Argus::ISensorMode * sensor_properties;
  Argus::Size2D<uint32_t> res;
  for (auto dvc : camera_devices) {
    properties = Argus::interface_cast<Argus::ICameraProperties>(dvc);
    model_string = properties->getModuleString();
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "%d %s", dvc_idx++, model_string.c_str());
    RCLCPP_INFO(this->get_logger(), "Sensor modes supported for this camera device:");
    status = properties->getAllSensorModes(&sensor_modes);
    if (status != Argus::STATUS_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get sensor modes vector");
      return;
    }
    snsr_idx = 0;
    for (auto snsr : sensor_modes) {
      sensor_properties = Argus::interface_cast<Argus::ISensorMode>(snsr);
      res = sensor_properties->getResolution();
      RCLCPP_INFO(this->get_logger(), "%d (%d x %d)", snsr_idx++, res.width(), res.height());
    }
  }

  // Get the parameter value for camera device and get its properties
  this->get_parameter("device", device_index);
  Argus::CameraDevice * device = camera_devices[device_index];
  Argus::ICameraProperties * camera_properties = Argus::interface_cast<Argus::ICameraProperties>(
    device);
  if (!camera_properties) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get ICameraProperties interface\n");
    return;
  }

  // Get the parameter value for sensor model
  this->get_parameter("sensor", sensor_index);
  Argus::SensorMode * sensor_mode = sensor_modes[sensor_index];
  Argus::ISensorMode * i_sensor_mode = Argus::interface_cast<Argus::ISensorMode>(sensor_mode);
  if (!i_sensor_mode) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get sensor mode interface");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(
    this->get_logger(), "Capturing from device %d using sensor mode %d (%dx%d)",
    device_index, sensor_index,
    i_sensor_mode->getResolution().width(),
    i_sensor_mode->getResolution().height());

  // Geting camera calibration data and putting it into sensor_msgs.camera_info
  const Argus::Ext::ISyncSensorCalibrationData * sync_sensor_calibration_data =
    Argus::interface_cast<const Argus::Ext::ISyncSensorCalibrationData>(device);
  camerainfo_ = sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo());
  image_ = sensor_msgs::msg::Image::SharedPtr(new sensor_msgs::msg::Image());
  if (sync_sensor_calibration_data) {
    getCalibrationData(sync_sensor_calibration_data, camerainfo_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Cannot get ISyncSensorCalibrationData interface\n");
  }

  Argus::UniqueObj<Argus::CaptureSession> capture_session(camera_provider->createCaptureSession(
      device, &status));

  if (status != Argus::STATUS_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create capture session\n");
    return;
  }

  Argus::ICaptureSession * iSession =
    Argus::interface_cast<Argus::ICaptureSession>(capture_session);
  if (!iSession) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get Capture Session interface\n");
    return;
  }

  Argus::UniqueObj<Argus::OutputStreamSettings> stream_settings(
    iSession->createOutputStreamSettings(Argus::STREAM_TYPE_BUFFER));
  Argus::IBufferOutputStreamSettings * i_stream_settings =
    Argus::interface_cast<Argus::IBufferOutputStreamSettings>(stream_settings);
  if (!i_stream_settings) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get IBufferOutputStreamSettings interface\n");
    return;
  }

  i_stream_settings->setBufferType(Argus::BUFFER_TYPE_EGL_IMAGE);
  if (i_stream_settings->setSyncType(Argus::SYNC_TYPE_NONE) != Argus::STATUS_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to disable EGL sync");
    return;
  }

  Argus::UniqueObj<Argus::OutputStream> output_stream(iSession->createOutputStream(
      stream_settings.get()));
  if (!output_stream) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create OutputStream\n");
    return;
  }
  Argus::IBufferOutputStream * i_buffer_output_stream =
    Argus::interface_cast<Argus::IBufferOutputStream>(output_stream);
  if (!i_buffer_output_stream) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create IBufferOutputStream interface\n");
    return;
  }

  Argus::Size2D<uint32_t> size =
    Argus::Size2D<uint32_t>(
    (i_sensor_mode->getResolution()).width(),
    (i_sensor_mode->getResolution()).height() );

  EGLint major, minor;
  EGLDisplay egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  eglInitialize(egl_display, &major, &minor);

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

  ArgusSamples::NativeBuffer * native_buffers[NUM_BUFFERS];
  EGLImageKHR egl_images[NUM_BUFFERS];
  Argus::UniqueObj<Argus::Buffer> buffers[NUM_BUFFERS];

  ArgusSamples::NvNativeBuffer * buf;

  i_buffer_settings->setEGLDisplay(egl_display);

  // cppcheck-suppress syntaxError
  for (uint32_t j = 0; j < NUM_BUFFERS; j++) {
    buf = ArgusSamples::NvNativeBuffer::create(size, NvBufferColorFormat_NV12);
    native_buffers[j] = (ArgusSamples::NativeBuffer *)buf;
    if (!native_buffers[j]) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create native buffer\n");
      return;
    }

    egl_images[j] = native_buffers[j]->createEGLImage(egl_display);
    if (egl_images[j] == EGL_NO_IMAGE_KHR) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create eglImage from native buffer\n");
      return;
    }

    i_buffer_settings->setEGLImage(egl_images[j]);

    buffers[j].reset(i_buffer_output_stream->createBuffer(buffer_settings.get()));

    if (!Argus::interface_cast<Argus::IEGLImageBuffer>(buffers[j])) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create buffer\n");
      return;
    }
    if (i_buffer_output_stream->releaseBuffer(buffers[j].get()) != Argus::STATUS_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to release buffer for capture use\n");
      return;
    }
  }

  std::string encoding;
  this->get_parameter("output_encoding", encoding);

  Argus::UniqueObj<Argus::Request> request(iSession->createRequest());
  Argus::IRequest * i_request = Argus::interface_cast<Argus::IRequest>(request);
  if (!i_request) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get capture request interface\n");
    return;
  }

  status = i_request->enableOutputStream(output_stream.get());
  if (status != Argus::STATUS_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable stream in capture request\n");
    return;
  }

  Argus::ISourceSettings * source_settings =
    Argus::interface_cast<Argus::ISourceSettings>(request);
  if (!source_settings) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get source settings request interface\n");
    return;
  }

  source_settings->setSensorMode(sensor_mode);
  width_ = (i_sensor_mode->getResolution()).width();
  height_ = (i_sensor_mode->getResolution()).height();
  MonocularConsumerThread * mono_consumer_thread = new MonocularConsumerThread(
    this,
    output_stream.get());

  if (!(mono_consumer_thread->initialize())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Consumer Thread\n");
    return;
  }
  if (!(mono_consumer_thread->waitRunning())) {
    RCLCPP_ERROR(this->get_logger(), "Consumer Thread does not enter running state\n");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Starting repeat capture requests.");
  if (iSession->repeat(request.get()) != Argus::STATUS_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start repeat capture request for preview");
    return;
  }

  while (rclcpp::ok()) {
  }

  iSession->stopRepeat();
  iSession->waitForIdle();

  for (uint32_t j = 0; j < NUM_BUFFERS; j++) {
    delete native_buffers[j];
    buffers[j].reset();
  }
  RCLCPP_INFO(this->get_logger(), "Argus shutting down");
  buffer_settings.reset();
  output_stream.reset();
  stream_settings.reset();
  capture_session.reset();
  cameraProvider.reset();
}

MonocularNode::~MonocularNode()
{
}

}  // namespace argus
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros::argus::MonocularNode)
