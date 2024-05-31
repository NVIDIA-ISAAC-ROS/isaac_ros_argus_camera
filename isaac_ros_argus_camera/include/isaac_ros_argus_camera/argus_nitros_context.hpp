// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_ARGUS_CAMERA__ARGUS_NITROS_CONTEXT_HPP_
#define ISAAC_ROS_ARGUS_CAMERA__ARGUS_NITROS_CONTEXT_HPP_

#include <memory>
#include <mutex>

#include "isaac_ros_nitros/nitros_context.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace argus
{

// Global NitrosContext for data argus
extern std::unique_ptr<nvidia::isaac_ros::nitros::NitrosContext> g_argus_nitros_context;

// Mutex for the argus' global NitrosContext
extern std::mutex g_argus_nitros_context_mutex;

// Is the global argus' global NitrosContext initialized?
extern bool g_argus_nitros_context_initialized;

// Is the global argus' global NitrosContext destroyed?
extern bool g_argus_nitros_context_destroyed;

// Get the argus' global NitrosContext object
// If not initialized, the first call creates an NitrosContext object and starts
// the argus' graph
nvidia::isaac_ros::nitros::NitrosContext & GetArgusNitrosContext();

// Terminate the argus' graph and release the context
void DestroyArgusNitrosContext();

}  // namespace argus
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_ARGUS_CAMERA__ARGUS_NITROS_CONTEXT_HPP_
