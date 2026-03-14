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

#include "isaac-handtracking-plugin/IsaacSimHandTrackerCAPI.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

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

/// Minimal WebSocket client for communicating with the Haply SDK service on localhost.
/// Supports text-frame read/write, ping/pong, and clean close over unencrypted TCP.
class HaplyWebSocket
{
public:
    HaplyWebSocket();
    ~HaplyWebSocket();

    HaplyWebSocket(const HaplyWebSocket&) = delete;
    HaplyWebSocket& operator=(const HaplyWebSocket&) = delete;

    /// Connect and perform the HTTP upgrade handshake.
    /// @return true on success.
    bool connect(const std::string& host, uint16_t port, const std::string& path = "/");

    /// Send a UTF-8 text frame.
    bool send_text(const std::string& payload);

    /// Receive a complete text frame (blocks until one arrives or error/close).
    /// @return true on success; the payload is written to @p out.
    bool recv_text(std::string& out);

    /// Perform a clean WebSocket close handshake and tear down the socket.
    void close();

    /// @return true when the underlying socket is connected.
    bool is_connected() const;

private:
    bool send_raw(const void* data, size_t len);
    bool recv_raw(void* data, size_t len);
    bool send_frame(uint8_t opcode, const void* payload, size_t len);

    int fd_ = -1;
};


/// 3-component vector (position / velocity / force).
struct HaplyVec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/// Quaternion orientation (w, x, y, z).
struct HaplyQuat
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/// Button state snapshot from a VerseGrip controller.
struct HaplyButtons
{
    bool button_0 = false;
    bool button_1 = false;
    bool button_2 = false;
    bool button_3 = false;
};

/// Aggregated, mutex-guarded state received from the Haply SDK.
struct HaplyDeviceState
{
    // Inverse3
    std::string inverse3_device_id;
    HaplyVec3 cursor_position;
    HaplyVec3 cursor_velocity;

    // VerseGrip
    std::string versegrip_device_id;
    HaplyQuat orientation;
    HaplyButtons buttons;

    // Handedness detected from the first config message ("left" or "right").
    // Defaults to "right" when not yet received.
    std::string handedness = "right";

    // True once we have received at least one message from the Haply SDK.
    bool has_data = false;
};


class ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT IsaacSimHaplyTracker
{
public:
    IsaacSimHaplyTracker();
    ~IsaacSimHaplyTracker();

    /// Initialize the tracker: start the background WebSocket thread.
    /// @param ws_host  WebSocket host (default "127.0.0.1").
    /// @param ws_port  WebSocket port (default 10001).
    /// @return true on success.
    bool initialize(const std::string& ws_host = "127.0.0.1", uint16_t ws_port = 10001);

    /// Fill the caller-provided joint-pose buffer with the latest Haply data.
    /// @param out_joint_poses  Buffer of size (ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT).
    /// @param out_joint_pose_count  Number of elements in the buffer.
    /// @return true when valid data was written.
    bool get_hand_data(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count);

    /// Stop the background thread, close the WebSocket, and release resources.
    void cleanup();

    /// @return a snapshot of the latest raw device state (thread-safe copy).
    HaplyDeviceState get_raw_state();

private:
    void io_loop(const std::string& host, uint16_t port);

    std::thread io_thread_;
    std::atomic<bool> running_{ false };

    mutable std::mutex state_mutex_;
    HaplyDeviceState state_;

    // Smoothed grip interpolant [0, 1] used for synthesised finger poses.
    float grip_interpolant_ = 0.0f;
};

} // namespace input_devices
} // namespace xr
} // namespace isaacsim
