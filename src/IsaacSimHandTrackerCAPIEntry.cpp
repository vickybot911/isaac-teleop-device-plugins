// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
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

#include "isaac-handtracking-plugin/IsaacSimHandTrackerCAPI.h"
#include "isaac-handtracking-plugin/IsaacSimHandTrackerRegistration.hpp"

#include <mutex>

namespace
{
using InitializeFunc = bool (*)(void);
using GetDataFunc = bool (*)(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count);
using ShutdownFunc = void (*)(void);

std::mutex g_mutex;
InitializeFunc g_initialize_ptr = nullptr;
GetDataFunc g_get_data_ptr = nullptr;
ShutdownFunc g_shutdown_ptr = nullptr;
} // namespace

namespace IsaacSimHandTracker
{
namespace detail
{
void Register(InitializeFunc initialize_func, GetDataFunc get_data_func, ShutdownFunc shutdown_func) noexcept
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_initialize_ptr = initialize_func;
    g_get_data_ptr = get_data_func;
    g_shutdown_ptr = shutdown_func;
}
} // namespace detail
} // namespace IsaacSimHandTracker

extern "C"
{

    ISAACSIM_HANDTRACKER_API bool IsaacSimHandTracker_Initialize(void)
    {
        InitializeFunc f;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            f = g_initialize_ptr;
        }
        return f ? f() : false;
    }

    ISAACSIM_HANDTRACKER_API bool IsaacSimHandTracker_GetData(IsaacSimHandJointPose* out_joint_poses,
                                                              int out_joint_pose_count)
    {
        GetDataFunc f;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            f = g_get_data_ptr;
        }
        return f ? f(out_joint_poses, out_joint_pose_count) : false;
    }

    ISAACSIM_HANDTRACKER_API void IsaacSimHandTracker_Shutdown(void)
    {
        ShutdownFunc f;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            f = g_shutdown_ptr;
        }
        if (f)
        {
            f();
        }
    }

} // extern "C"
