// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "extensions/utils/stereo_camera_synchronizer.hpp"
#include "gems/gxf_helpers/expected_macro.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

gxf_result_t StereoCameraSynchronizer::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;
  result &= registrar->parameter(
      rx_right_camera_, "rx_right_camera", "RX Right Camera Message",
      "Input for Right CameraMessage entities");
  result &= registrar->parameter(
      rx_left_camera_, "rx_left_camera", "RX Left Camera Message",
      "Input for Left CameraMessage entities");
  result &= registrar->parameter(
      tx_right_camera_, "tx_right_camera", "TX Right Camera Message",
      "Input for Right CameraMessage entities");
  result &= registrar->parameter(
      tx_left_camera_, "tx_left_camera", "TX Left Camera Message",
      "Input for Left CameraMessage entities");
  result &= registrar->parameter(
      max_timestamp_diff_, "max_timestamp_diff", "Max difference timestamps nanoseconds",
      "Max difference between timestamps in nanoseconds",
      static_cast<int64_t>(20000));  // 20 micro seconds
  return gxf::ToResultCode(result);
}

gxf_result_t StereoCameraSynchronizer::tick() {
  auto left_camera_entity = GXF_UNWRAP_OR_RETURN(rx_left_camera_->receive());
  auto right_camera_entity = GXF_UNWRAP_OR_RETURN(rx_right_camera_->receive());
  auto left_timestamp = GXF_UNWRAP_OR_RETURN(left_camera_entity.get<gxf::Timestamp>());
  auto right_timestamp = GXF_UNWRAP_OR_RETURN(right_camera_entity.get<gxf::Timestamp>());

  const int64_t diff = std::abs(left_timestamp->acqtime - right_timestamp->acqtime);
  if (diff > max_timestamp_diff_) {
    GXF_LOG_WARNING("L/R frames are not synchronized, timestamp diff = %ld ns", diff);
  }
  // Set right input timestamp to left input timestamp
  right_timestamp->acqtime = left_timestamp->acqtime;

  GXF_RETURN_IF_ERROR(tx_left_camera_->publish(left_camera_entity));
  GXF_RETURN_IF_ERROR(tx_right_camera_->publish(right_camera_entity));

  return GXF_SUCCESS;
}

}  // namespace isaac
}  // namespace nvidia
