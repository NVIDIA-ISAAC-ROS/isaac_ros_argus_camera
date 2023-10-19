# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

project(gxf_utils LANGUAGES C CXX)

# Dependencies
find_package(CUDAToolkit REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GXF ${ISAAC_ROS_GXF_VERSION} MODULE REQUIRED
    COMPONENTS
    std
    multimedia
    isaac_messages
)
find_package(yaml-cpp)

# Occupancy Grid Projector extension
add_library(gxf_utils SHARED
    extensions/utils/utils.cpp
    extensions/utils/stereo_camera_synchronizer.cpp
    extensions/utils/stereo_camera_synchronizer.hpp
)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
target_link_libraries(gxf_utils
  PUBLIC
    Eigen3::Eigen
    GXF::std
    GXF::multimedia
    yaml-cpp
    GXF::isaac_messages
)