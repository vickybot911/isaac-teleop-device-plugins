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
#ifndef ISAACSIM_HAND_TRACKER_REGISTRATION_HPP
#define ISAACSIM_HAND_TRACKER_REGISTRATION_HPP

#ifdef __cplusplus

#    include "isaac-handtracking-plugin/IsaacSimHandTrackerCAPI.h"

namespace IsaacSimHandTracker
{
namespace detail
{
using InitializeFunc = bool (*)(void);
using GetDataFunc = bool (*)(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count);
using ShutdownFunc = void (*)(void);

// Implemented in the library; binds the C entry points to these function pointers.
__attribute__((visibility("default"))) void Register(InitializeFunc initialize_func,
                                                     GetDataFunc get_data_func,
                                                     ShutdownFunc shutdown_func) noexcept;

template <typename Implementation>
struct Registrar
{
    Registrar()
    {
        Register(&Registrar::InitializeThunk, &Registrar::GetDataThunk, &Registrar::ShutdownThunk);
    }

    static bool InitializeThunk()
    {
        return Implementation::Initialize();
    }

    static bool GetDataThunk(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count)
    {
        return Implementation::GetData(out_joint_poses, out_joint_pose_count);
    }

    static void ShutdownThunk()
    {
        Implementation::Shutdown();
    }
};
} // namespace detail
} // namespace IsaacSimHandTracker

// Helper macro to register an implementation type that exposes:
//   static bool Initialize();
//   static bool GetData(IsaacSimHandJointPose*, int);
//   static void Shutdown();
#    define ISAACSIM_HANDTRACKER_REGISTER(ImplType)                                                                    \
        static ::IsaacSimHandTracker::detail::Registrar<ImplType> g_isaacsim_handtracker_registrar_instance_;

#endif // __cplusplus

#endif // ISAACSIM_HAND_TRACKER_REGISTRATION_HPP
