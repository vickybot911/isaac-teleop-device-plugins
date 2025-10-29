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

#include "ManusHandTrackingPlugin.h"

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"
#include "isaac-handtracking-plugin/IsaacSimHandTrackerCAPI.h"
#include "isaac-handtracking-plugin/IsaacSimHandTrackerRegistration.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace xr
{
namespace input_devices
{
class IsaacSimManusTracker;
}
} // namespace xr
} // namespace isaacsim
namespace
{
using isaacsim::xr::input_devices::IsaacSimManusTracker;
static std::unique_ptr<IsaacSimManusTracker> g_tracker_instance;
static std::mutex g_tracker_mutex;
} // namespace

struct ManusHandTrackingPlugin
{
    static bool Initialize()
    {
        // Create singleton tracker instance and initialize
        {
            std::lock_guard<std::mutex> lock(g_tracker_mutex);
            if (!g_tracker_instance)
            {
                g_tracker_instance = std::make_unique<IsaacSimManusTracker>();
                if (!g_tracker_instance->initialize())
                {
                    g_tracker_instance.reset();
                    return false;
                }
            }
        }
        return true;
    }

    static bool GetData(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count)
    {
        const int required_count = ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT;
        if (!out_joint_poses || out_joint_pose_count < required_count)
        {
            return false;
        }
        IsaacSimManusTracker* tracker = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_tracker_mutex);
            tracker = g_tracker_instance.get();
        }
        if (!tracker)
        {
            return false;
        }

        const auto glove_data = tracker->get_glove_data();

        const std::string hand_prefixes[ISAACSIM_HAND_COUNT] = { "left", "right" };
        bool any_available = false;

        for (int hand_index = 0; hand_index < ISAACSIM_HAND_COUNT; ++hand_index)
        {
            const std::string pos_key = hand_prefixes[hand_index] + std::string("_position");
            const std::string orient_key = hand_prefixes[hand_index] + std::string("_orientation");

            const std::vector<float>* pos = nullptr;
            const std::vector<float>* orient = nullptr;

            if (auto itp = glove_data.find(pos_key); itp != glove_data.end())
                pos = &itp->second;
            if (auto ito = glove_data.find(orient_key); ito != glove_data.end())
                orient = &ito->second;

            const int available_joints_from_pos = pos ? static_cast<int>(pos->size() / 3) : 0;
            const int available_joints_from_orient = orient ? static_cast<int>(orient->size() / 4) : 0;
            const int available =
                std::min({ ISAACSIM_HAND_JOINT_COUNT, available_joints_from_pos, available_joints_from_orient });
            any_available = any_available || (available > 0);

            for (int joint_index = 0; joint_index < ISAACSIM_HAND_JOINT_COUNT; ++joint_index)
            {
                const int out_index = hand_index * ISAACSIM_HAND_JOINT_COUNT + joint_index;
                if (joint_index < available)
                {
                    out_joint_poses[out_index].position[0] = (*pos)[joint_index * 3 + 0];
                    out_joint_poses[out_index].position[1] = (*pos)[joint_index * 3 + 1];
                    out_joint_poses[out_index].position[2] = (*pos)[joint_index * 3 + 2];
                    out_joint_poses[out_index].orientation[0] = (*orient)[joint_index * 4 + 1];
                    out_joint_poses[out_index].orientation[1] = (*orient)[joint_index * 4 + 2];
                    out_joint_poses[out_index].orientation[2] = (*orient)[joint_index * 4 + 3];
                    out_joint_poses[out_index].orientation[3] = (*orient)[joint_index * 4 + 0];
                    out_joint_poses[out_index].radius = 0.01f;
                    out_joint_poses[out_index].locationFlags = ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID |
                                                               ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID;
                }
                else
                {
                    out_joint_poses[out_index].position[0] = 0.0f;
                    out_joint_poses[out_index].position[1] = 0.0f;
                    out_joint_poses[out_index].position[2] = 0.0f;
                    out_joint_poses[out_index].orientation[0] = 0.0f;
                    out_joint_poses[out_index].orientation[1] = 0.0f;
                    out_joint_poses[out_index].orientation[2] = 0.0f;
                    out_joint_poses[out_index].orientation[3] = 1.0f;
                    out_joint_poses[out_index].radius = 0.01f;
                    out_joint_poses[out_index].locationFlags = 0;
                }
            }
        }
        return any_available;
    }

    static void Shutdown()
    {
        std::unique_ptr<IsaacSimManusTracker> to_cleanup;
        {
            std::lock_guard<std::mutex> lock(g_tracker_mutex);
            to_cleanup.swap(g_tracker_instance);
        }
        if (to_cleanup)
        {
            to_cleanup->cleanup();
        }
    }
};

namespace isaacsim
{
namespace xr
{
namespace input_devices
{

IsaacSimManusTracker* IsaacSimManusTracker::s_instance = nullptr;
std::mutex IsaacSimManusTracker::s_instance_mutex;

IsaacSimManusTracker::IsaacSimManusTracker() = default;

IsaacSimManusTracker::~IsaacSimManusTracker()
{
    cleanup();
}

bool IsaacSimManusTracker::initialize()
{
    {
        std::lock_guard<std::mutex> lock(s_instance_mutex);
        if (s_instance != nullptr)
        {
            std::cerr << "ManusTracker instance already exists - only one instance allowed" << std::endl;
            return false;
        }
        s_instance = this;
    }

    std::cout << "Initializing Manus SDK..." << std::endl;
    const SDKReturnCode t_InitializeResult = CoreSdk_InitializeIntegrated();
    if (t_InitializeResult != SDKReturnCode::SDKReturnCode_Success)
    {
        std::cerr << "Failed to initialize Manus SDK, error code: " << static_cast<int>(t_InitializeResult) << std::endl;
        std::lock_guard<std::mutex> lock(s_instance_mutex);
        s_instance = nullptr;
        return false;
    }
    std::cout << "Manus SDK initialized successfully" << std::endl;

    RegisterCallbacks();

    CoordinateSystemVUH t_VUH;
    CoordinateSystemVUH_Init(&t_VUH);
    t_VUH.handedness = Side::Side_Right;
    t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
    t_VUH.view = AxisView::AxisView_XFromViewer;
    t_VUH.unitScale = 1.0f;

    std::cout << "Setting up coordinate system (Z-up, right-handed, meters)..." << std::endl;
    const SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true);

    if (t_CoordinateResult != SDKReturnCode::SDKReturnCode_Success)
    {
        std::cerr
            << "Failed to initialize Manus SDK coordinate system, error code: " << static_cast<int>(t_CoordinateResult)
            << std::endl;
        std::lock_guard<std::mutex> lock(s_instance_mutex);
        s_instance = nullptr;
        return false;
    }
    std::cout << "Coordinate system initialized successfully" << std::endl;

    ConnectToGloves();
    return true;
}

std::unordered_map<std::string, std::vector<float>> IsaacSimManusTracker::get_glove_data()
{
    std::lock_guard<std::mutex> lock(output_map_mutex);
    return output_map;
}

void IsaacSimManusTracker::cleanup()
{
    std::lock_guard<std::mutex> lock(s_instance_mutex);
    if (s_instance == this)
    {
        CoreSdk_RegisterCallbackForRawSkeletonStream(nullptr);
        CoreSdk_RegisterCallbackForLandscapeStream(nullptr);
        CoreSdk_RegisterCallbackForErgonomicsStream(nullptr);
        DisconnectFromGloves();
        CoreSdk_ShutDown();
        s_instance = nullptr;
    }
}

void IsaacSimManusTracker::RegisterCallbacks()
{
    CoreSdk_RegisterCallbackForRawSkeletonStream(OnSkeletonStream);
    CoreSdk_RegisterCallbackForLandscapeStream(OnLandscapeStream);
    CoreSdk_RegisterCallbackForErgonomicsStream(OnErgonomicsStream);
}

void IsaacSimManusTracker::ConnectToGloves()
{
    bool connected = false;
    const int max_attempts = 30; // Maximum connection attempts
    const auto retry_delay = std::chrono::milliseconds(1000); // 1 second delay between attempts
    int attempts = 0;

    std::cout << "Looking for Manus gloves..." << std::endl;

    while (!connected && attempts < max_attempts)
    {
        attempts++;

        if (const auto start_result = CoreSdk_LookForHosts(1, false); start_result != SDKReturnCode::SDKReturnCode_Success)
        {
            std::cerr << "Failed to look for hosts (attempt " << attempts << "/" << max_attempts << ")" << std::endl;
            std::this_thread::sleep_for(retry_delay);
            continue;
        }

        uint32_t number_of_hosts_found{};
        if (const auto number_result = CoreSdk_GetNumberOfAvailableHostsFound(&number_of_hosts_found);
            number_result != SDKReturnCode::SDKReturnCode_Success)
        {
            std::cerr << "Failed to get number of available hosts (attempt " << attempts << "/" << max_attempts << ")"
                      << std::endl;
            std::this_thread::sleep_for(retry_delay);
            continue;
        }

        if (number_of_hosts_found == 0)
        {
            std::cerr << "Failed to find hosts (attempt " << attempts << "/" << max_attempts << ")" << std::endl;
            std::this_thread::sleep_for(retry_delay);
            continue;
        }

        std::vector<ManusHost> available_hosts(number_of_hosts_found);

        if (const auto hosts_result = CoreSdk_GetAvailableHostsFound(available_hosts.data(), number_of_hosts_found);
            hosts_result != SDKReturnCode::SDKReturnCode_Success)
        {
            std::cerr << "Failed to get available hosts (attempt " << attempts << "/" << max_attempts << ")" << std::endl;
            std::this_thread::sleep_for(retry_delay);
            continue;
        }

        if (const auto connect_result = CoreSdk_ConnectToHost(available_hosts[0]);
            connect_result == SDKReturnCode::SDKReturnCode_NotConnected)
        {
            std::cerr << "Failed to connect to host (attempt " << attempts << "/" << max_attempts << ")" << std::endl;
            std::this_thread::sleep_for(retry_delay);
            continue;
        }

        connected = true;
        is_connected = true;
        std::cout << "Successfully connected to Manus host after " << attempts << " attempts" << std::endl;
    }

    if (!connected)
    {
        std::cerr << "Failed to connect to Manus gloves after " << max_attempts << " attempts" << std::endl;
        throw std::runtime_error("Failed to connect to Manus gloves");
    }
}

void IsaacSimManusTracker::DisconnectFromGloves()
{
    if (is_connected)
    {
        CoreSdk_Disconnect();
        is_connected = false;
        std::cout << "Disconnected from Manus gloves" << std::endl;
    }
}

void IsaacSimManusTracker::OnSkeletonStream(const SkeletonStreamInfo* skeleton_stream_info)
{
    std::lock_guard<std::mutex> instance_lock(s_instance_mutex);
    if (!s_instance)
    {
        return;
    }

    std::lock_guard<std::mutex> output_lock(s_instance->output_map_mutex);

    for (uint32_t i = 0; i < skeleton_stream_info->skeletonsCount; i++)
    {
        RawSkeletonInfo skeleton_info;
        CoreSdk_GetRawSkeletonInfo(i, &skeleton_info);

        std::vector<SkeletonNode> nodes(skeleton_info.nodesCount);
        skeleton_info.publishTime = skeleton_stream_info->publishTime;
        CoreSdk_GetRawSkeletonData(i, nodes.data(), skeleton_info.nodesCount);

        uint32_t glove_id = skeleton_info.gloveId;

        // Check if glove ID matches any known glove
        bool is_left_glove, is_right_glove;
        {
            std::lock_guard<std::mutex> landscape_lock(s_instance->landscape_mutex);
            is_left_glove = s_instance->left_glove_id && glove_id == *s_instance->left_glove_id;
            is_right_glove = s_instance->right_glove_id && glove_id == *s_instance->right_glove_id;
        }

        if (!is_left_glove && !is_right_glove)
        {
            std::cerr << "Skipping data from unknown glove ID: " << glove_id << std::endl;
            continue;
        }

        std::string prefix = is_left_glove ? "left" : "right";

        // Store position data (3 floats per node: x, y, z)
        std::string pos_key = prefix + "_position";
        s_instance->output_map[pos_key].resize(skeleton_info.nodesCount * 3);

        // Store orientation data (4 floats per node: w, x, y, z)
        std::string orient_key = prefix + "_orientation";
        s_instance->output_map[orient_key].resize(skeleton_info.nodesCount * 4);

        for (uint32_t j = 0; j < skeleton_info.nodesCount; j++)
        {
            const auto& position = nodes[j].transform.position;
            s_instance->output_map[pos_key][j * 3 + 0] = position.x;
            s_instance->output_map[pos_key][j * 3 + 1] = position.y;
            s_instance->output_map[pos_key][j * 3 + 2] = position.z;

            const auto& orientation = nodes[j].transform.rotation;
            s_instance->output_map[orient_key][j * 4 + 0] = orientation.w;
            s_instance->output_map[orient_key][j * 4 + 1] = orientation.x;
            s_instance->output_map[orient_key][j * 4 + 2] = orientation.y;
            s_instance->output_map[orient_key][j * 4 + 3] = orientation.z;
        }
    }
}

void IsaacSimManusTracker::OnLandscapeStream(const Landscape* landscape)
{
    std::lock_guard<std::mutex> instance_lock(s_instance_mutex);
    if (!s_instance)
    {
        return;
    }

    const auto& gloves = landscape->gloveDevices;

    std::lock_guard<std::mutex> landscape_lock(s_instance->landscape_mutex);

    // We only support one left and one right glove
    if (gloves.gloveCount > 2)
    {
        std::cerr << "Invalid number of gloves detected: " << gloves.gloveCount << std::endl;
        return;
    }

    // Extract glove IDs from landscape data
    for (uint32_t i = 0; i < gloves.gloveCount; i++)
    {
        const GloveLandscapeData& glove = gloves.gloves[i];
        if (glove.side == Side::Side_Left)
        {
            s_instance->left_glove_id = glove.id;
        }
        else if (glove.side == Side::Side_Right)
        {
            s_instance->right_glove_id = glove.id;
        }
    }
}

void IsaacSimManusTracker::OnErgonomicsStream(const ErgonomicsStream* ergonomics_stream)
{
    std::lock_guard<std::mutex> instance_lock(s_instance_mutex);
    if (!s_instance)
    {
        return;
    }

    std::lock_guard<std::mutex> output_lock(s_instance->output_map_mutex);

    for (uint32_t i = 0; i < ergonomics_stream->dataCount; i++)
    {
        if (ergonomics_stream->data[i].isUserID)
            continue;

        uint32_t glove_id = ergonomics_stream->data[i].id;

        // Check if glove ID matches any known glove
        bool is_left_glove, is_right_glove;
        {
            std::lock_guard<std::mutex> landscape_lock(s_instance->landscape_mutex);
            is_left_glove = s_instance->left_glove_id && glove_id == *s_instance->left_glove_id;
            is_right_glove = s_instance->right_glove_id && glove_id == *s_instance->right_glove_id;
        }

        if (!is_left_glove && !is_right_glove)
        {
            std::cerr << "Skipping ergonomics data from unknown glove ID: " << glove_id << std::endl;
            continue;
        }

        std::string prefix = is_left_glove ? "left" : "right";
        std::string angle_key = prefix + "_angle";
        s_instance->output_map[angle_key].clear();
        s_instance->output_map[angle_key].reserve(ErgonomicsDataType_MAX_SIZE);

        for (int j = 0; j < ErgonomicsDataType_MAX_SIZE; j++)
        {
            float value = ergonomics_stream->data[i].data[j];
            s_instance->output_map[angle_key].push_back(value);
        }
    }
}

} // namespace input_devices
} // namespace xr
} // namespace isaacsim

// Register the implementation so the C API forwards to it.
ISAACSIM_HANDTRACKER_REGISTER(ManusHandTrackingPlugin)
