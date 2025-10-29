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
#pragma once

#include "ManusSDK.h"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _MSC_VER
#    if ISAACSIM_XR_INPUT_DEVICES_EXPORT
#        define ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT __declspec(dllexport)
#    else
#        define ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT __declspec(dllimport)
#    endif
#else
#    define ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT __attribute__((visibility("default")))
#endif

namespace isaacsim
{
namespace xr
{
namespace input_devices
{

class ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT IsaacSimManusTracker
{
public:
    IsaacSimManusTracker();
    ~IsaacSimManusTracker();

    bool initialize();
    std::unordered_map<std::string, std::vector<float>> get_glove_data();
    void cleanup();

private:
    static IsaacSimManusTracker* s_instance;
    static std::mutex s_instance_mutex;

    // ManusSDK specific members
    void RegisterCallbacks();
    void ConnectToGloves();
    void DisconnectFromGloves();

    // Callback functions
    static void OnSkeletonStream(const SkeletonStreamInfo* skeleton_stream_info);
    static void OnLandscapeStream(const Landscape* landscape);
    static void OnErgonomicsStream(const ErgonomicsStream* ergonomics_stream);

    std::mutex output_map_mutex;
    std::mutex landscape_mutex;
    std::unordered_map<std::string, std::vector<float>> output_map;
    std::optional<uint32_t> left_glove_id;
    std::optional<uint32_t> right_glove_id;
    bool is_connected = false;
};

} // namespace input_devices
} // namespace xr
} // namespace isaacsim
