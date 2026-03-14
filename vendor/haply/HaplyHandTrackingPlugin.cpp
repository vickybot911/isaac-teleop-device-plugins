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

#include "HaplyHandTrackingPlugin.h"

#include "isaac-handtracking-plugin/IsaacSimHandTrackerCAPI.h"
#include "isaac-handtracking-plugin/IsaacSimHandTrackerRegistration.hpp"
#include "third_party/nlohmann/json.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <netdb.h>
#include <netinet/tcp.h>
#include <random>
#include <sys/socket.h>
#include <unistd.h>

using json = nlohmann::json;

namespace isaacsim
{
namespace xr
{
namespace input_devices
{

// ============================================================================
// HaplyWebSocket implementation
// ============================================================================

HaplyWebSocket::HaplyWebSocket() = default;

HaplyWebSocket::~HaplyWebSocket()
{
    close();
}

bool HaplyWebSocket::is_connected() const
{
    return fd_ >= 0;
}

bool HaplyWebSocket::send_raw(const void* data, size_t len)
{
    const uint8_t* ptr = static_cast<const uint8_t*>(data);
    size_t sent = 0;
    while (sent < len)
    {
        ssize_t n = ::send(fd_, ptr + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0)
        {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool HaplyWebSocket::recv_raw(void* data, size_t len)
{
    uint8_t* ptr = static_cast<uint8_t*>(data);
    size_t received = 0;
    while (received < len)
    {
        ssize_t n = ::recv(fd_, ptr + received, len - received, 0);
        if (n <= 0)
        {
            return false;
        }
        received += static_cast<size_t>(n);
    }
    return true;
}

bool HaplyWebSocket::connect(const std::string& host, uint16_t port, const std::string& path)
{
    close();

    // Resolve host
    struct addrinfo hints
    {
    };
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* res = nullptr;
    std::string port_str = std::to_string(port);
    if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || !res)
    {
        std::cerr << "[HaplyWebSocket] Failed to resolve host: " << host << std::endl;
        return false;
    }

    fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd_ < 0)
    {
        freeaddrinfo(res);
        std::cerr << "[HaplyWebSocket] Failed to create socket" << std::endl;
        return false;
    }

    if (::connect(fd_, res->ai_addr, res->ai_addrlen) < 0)
    {
        freeaddrinfo(res);
        ::close(fd_);
        fd_ = -1;
        std::cerr << "[HaplyWebSocket] Failed to connect to " << host << ":" << port << std::endl;
        return false;
    }
    freeaddrinfo(res);

    // Disable Nagle's algorithm for lower latency
    int flag = 1;
    setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // Generate a random 16-byte WebSocket key
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    uint8_t key_bytes[16];
    for (int i = 0; i < 16; ++i)
    {
        key_bytes[i] = static_cast<uint8_t>(dis(gen));
    }

    // Base64 encode the key (simple implementation)
    static const char* b64_table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string ws_key;
    ws_key.reserve(24);
    for (int i = 0; i < 16; i += 3)
    {
        uint32_t n = (static_cast<uint32_t>(key_bytes[i]) << 16);
        if (i + 1 < 16)
            n |= (static_cast<uint32_t>(key_bytes[i + 1]) << 8);
        if (i + 2 < 16)
            n |= static_cast<uint32_t>(key_bytes[i + 2]);
        ws_key += b64_table[(n >> 18) & 0x3F];
        ws_key += b64_table[(n >> 12) & 0x3F];
        ws_key += (i + 1 < 16) ? b64_table[(n >> 6) & 0x3F] : '=';
        ws_key += (i + 2 < 16) ? b64_table[n & 0x3F] : '=';
    }

    // Build HTTP upgrade request
    std::string request = "GET " + path + " HTTP/1.1\r\n";
    request += "Host: " + host + ":" + std::to_string(port) + "\r\n";
    request += "Upgrade: websocket\r\n";
    request += "Connection: Upgrade\r\n";
    request += "Sec-WebSocket-Key: " + ws_key + "\r\n";
    request += "Sec-WebSocket-Version: 13\r\n";
    request += "\r\n";

    if (!send_raw(request.data(), request.size()))
    {
        close();
        std::cerr << "[HaplyWebSocket] Failed to send HTTP upgrade request" << std::endl;
        return false;
    }

    // Read HTTP response headers (look for "101 Switching Protocols")
    std::string response;
    char buf[1];
    bool found_end = false;
    while (!found_end && response.size() < 4096)
    {
        if (!recv_raw(buf, 1))
        {
            close();
            std::cerr << "[HaplyWebSocket] Failed to receive HTTP upgrade response" << std::endl;
            return false;
        }
        response += buf[0];
        if (response.size() >= 4 && response.substr(response.size() - 4) == "\r\n\r\n")
        {
            found_end = true;
        }
    }

    if (response.find("101") == std::string::npos)
    {
        close();
        std::cerr << "[HaplyWebSocket] Upgrade failed, response: " << response.substr(0, 80) << std::endl;
        return false;
    }

    return true;
}

bool HaplyWebSocket::send_frame(uint8_t opcode, const void* payload, size_t len)
{
    if (fd_ < 0)
        return false;

    // Frame header
    std::vector<uint8_t> frame;
    frame.reserve(14 + len);

    // First byte: FIN + opcode
    frame.push_back(0x80 | opcode);

    // Second byte: MASK (clients must mask) + payload length
    if (len <= 125)
    {
        frame.push_back(0x80 | static_cast<uint8_t>(len));
    }
    else if (len <= 65535)
    {
        frame.push_back(0x80 | 126);
        frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(len & 0xFF));
    }
    else
    {
        frame.push_back(0x80 | 127);
        for (int i = 7; i >= 0; --i)
        {
            frame.push_back(static_cast<uint8_t>((len >> (8 * i)) & 0xFF));
        }
    }

    // 4-byte mask key
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    uint8_t mask[4];
    for (int i = 0; i < 4; ++i)
    {
        mask[i] = static_cast<uint8_t>(dis(gen));
        frame.push_back(mask[i]);
    }

    // Masked payload
    const uint8_t* src = static_cast<const uint8_t*>(payload);
    for (size_t i = 0; i < len; ++i)
    {
        frame.push_back(src[i] ^ mask[i % 4]);
    }

    return send_raw(frame.data(), frame.size());
}

bool HaplyWebSocket::send_text(const std::string& payload)
{
    return send_frame(0x01, payload.data(), payload.size());
}

bool HaplyWebSocket::recv_text(std::string& out)
{
    if (fd_ < 0)
        return false;

    out.clear();

    // We may receive a fragmented message; loop until FIN is set
    bool fin = false;
    while (!fin)
    {
        uint8_t header[2];
        if (!recv_raw(header, 2))
            return false;

        fin = (header[0] & 0x80) != 0;
        uint8_t opcode = header[0] & 0x0F;
        bool masked = (header[1] & 0x80) != 0;
        uint64_t payload_len = header[1] & 0x7F;

        if (payload_len == 126)
        {
            uint8_t ext[2];
            if (!recv_raw(ext, 2))
                return false;
            payload_len = (static_cast<uint64_t>(ext[0]) << 8) | ext[1];
        }
        else if (payload_len == 127)
        {
            uint8_t ext[8];
            if (!recv_raw(ext, 8))
                return false;
            payload_len = 0;
            for (int i = 0; i < 8; ++i)
            {
                payload_len = (payload_len << 8) | ext[i];
            }
        }

        uint8_t mask[4] = { 0, 0, 0, 0 };
        if (masked)
        {
            if (!recv_raw(mask, 4))
                return false;
        }

        std::vector<uint8_t> data(payload_len);
        if (payload_len > 0)
        {
            if (!recv_raw(data.data(), payload_len))
                return false;
            if (masked)
            {
                for (size_t i = 0; i < payload_len; ++i)
                {
                    data[i] ^= mask[i % 4];
                }
            }
        }

        // Handle control frames
        if (opcode == 0x08)
        {
            // Close frame - send close response and return false
            send_frame(0x08, data.data(), data.size());
            return false;
        }
        else if (opcode == 0x09)
        {
            // Ping - respond with pong
            send_frame(0x0A, data.data(), data.size());
            continue;
        }
        else if (opcode == 0x0A)
        {
            // Pong - ignore
            continue;
        }
        else if (opcode == 0x01 || opcode == 0x00)
        {
            // Text or continuation
            out.append(reinterpret_cast<char*>(data.data()), data.size());
        }
        else
        {
            // Binary or unknown - skip for now
            out.append(reinterpret_cast<char*>(data.data()), data.size());
        }
    }

    return true;
}

void HaplyWebSocket::close()
{
    if (fd_ >= 0)
    {
        // Send close frame (best effort)
        send_frame(0x08, nullptr, 0);
        ::close(fd_);
        fd_ = -1;
    }
}

// ============================================================================
// IsaacSimHaplyTracker implementation
// ============================================================================

IsaacSimHaplyTracker::IsaacSimHaplyTracker() = default;

IsaacSimHaplyTracker::~IsaacSimHaplyTracker()
{
    cleanup();
}

bool IsaacSimHaplyTracker::initialize(const std::string& ws_host, uint16_t ws_port)
{
    if (running_.load())
    {
        std::cerr << "[HaplyTracker] Already initialized" << std::endl;
        return false;
    }

    std::cout << "[HaplyTracker] Initializing, connecting to " << ws_host << ":" << ws_port << std::endl;

    running_.store(true);
    io_thread_ = std::thread(&IsaacSimHaplyTracker::io_loop, this, ws_host, ws_port);

    // Wait briefly for connection to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

void IsaacSimHaplyTracker::cleanup()
{
    if (running_.load())
    {
        running_.store(false);
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
        std::cout << "[HaplyTracker] Cleanup complete" << std::endl;
    }
}

HaplyDeviceState IsaacSimHaplyTracker::get_raw_state()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

void IsaacSimHaplyTracker::io_loop(const std::string& host, uint16_t port)
{
    HaplyWebSocket ws;
    int reconnect_delay_ms = 500;
    constexpr int max_reconnect_delay_ms = 10000;

    while (running_.load())
    {
        if (!ws.connect(host, port, "/"))
        {
            std::cerr << "[HaplyTracker] Connection failed, retrying in " << reconnect_delay_ms << " ms" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
            reconnect_delay_ms = std::min(reconnect_delay_ms * 2, max_reconnect_delay_ms);
            continue;
        }

        std::cout << "[HaplyTracker] Connected to Haply SDK" << std::endl;
        reconnect_delay_ms = 500;

        while (running_.load() && ws.is_connected())
        {
            std::string msg;
            if (!ws.recv_text(msg))
            {
                std::cerr << "[HaplyTracker] Connection lost" << std::endl;
                break;
            }

            // Parse JSON
            try
            {
                json j = json::parse(msg);

                std::lock_guard<std::mutex> lock(state_mutex_);

                // Parse inverse3 array
                if (j.contains("inverse3") && j["inverse3"].is_array() && !j["inverse3"].empty())
                {
                    const auto& inv3 = j["inverse3"][0];
                    if (inv3.contains("device_id"))
                    {
                        state_.inverse3_device_id = inv3["device_id"].get<std::string>();
                    }
                    // Handedness from config (only in first message)
                    if (inv3.contains("config") && inv3["config"].contains("handedness"))
                    {
                        state_.handedness = inv3["config"]["handedness"].get<std::string>();
                    }
                    if (inv3.contains("state"))
                    {
                        const auto& st = inv3["state"];
                        if (st.contains("cursor_position"))
                        {
                            const auto& pos = st["cursor_position"];
                            state_.cursor_position.x = pos.value("x", 0.0f);
                            state_.cursor_position.y = pos.value("y", 0.0f);
                            state_.cursor_position.z = pos.value("z", 0.0f);
                        }
                        if (st.contains("cursor_velocity"))
                        {
                            const auto& vel = st["cursor_velocity"];
                            state_.cursor_velocity.x = vel.value("x", 0.0f);
                            state_.cursor_velocity.y = vel.value("y", 0.0f);
                            state_.cursor_velocity.z = vel.value("z", 0.0f);
                        }
                    }
                    state_.has_data = true;
                }

                // Parse wireless_verse_grip array
                if (j.contains("wireless_verse_grip") && j["wireless_verse_grip"].is_array() &&
                    !j["wireless_verse_grip"].empty())
                {
                    const auto& vg = j["wireless_verse_grip"][0];
                    if (vg.contains("device_id"))
                    {
                        state_.versegrip_device_id = vg["device_id"].get<std::string>();
                    }
                    if (vg.contains("state"))
                    {
                        const auto& st = vg["state"];
                        if (st.contains("orientation"))
                        {
                            const auto& ori = st["orientation"];
                            state_.orientation.w = ori.value("w", 1.0f);
                            state_.orientation.x = ori.value("x", 0.0f);
                            state_.orientation.y = ori.value("y", 0.0f);
                            state_.orientation.z = ori.value("z", 0.0f);
                        }
                        if (st.contains("buttons"))
                        {
                            const auto& btn = st["buttons"];
                            state_.buttons.button_0 = btn.value("button_0", false);
                            state_.buttons.button_1 = btn.value("button_1", false);
                            state_.buttons.button_2 = btn.value("button_2", false);
                            state_.buttons.button_3 = btn.value("button_3", false);
                        }
                    }
                    state_.has_data = true;
                }

                // Send a command to keep receiving updates (set zero force)
                if (!state_.inverse3_device_id.empty())
                {
                    json cmd;
                    cmd["inverse3"] = json::array();
                    json dev;
                    dev["device_id"] = state_.inverse3_device_id;
                    dev["commands"]["set_cursor_force"]["values"]["x"] = 0;
                    dev["commands"]["set_cursor_force"]["values"]["y"] = 0;
                    dev["commands"]["set_cursor_force"]["values"]["z"] = 0;
                    cmd["inverse3"].push_back(dev);
                    ws.send_text(cmd.dump());
                }
            }
            catch (const json::exception& e)
            {
                std::cerr << "[HaplyTracker] JSON parse error: " << e.what() << std::endl;
            }
        }

        ws.close();
    }

    ws.close();
}

bool IsaacSimHaplyTracker::get_hand_data(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count)
{
    const int required_count = ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT;
    if (!out_joint_poses || out_joint_pose_count < required_count)
    {
        return false;
    }

    HaplyDeviceState snapshot;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot = state_;
    }

    if (!snapshot.has_data)
    {
        return false;
    }

    // Determine which hand to populate based on detected handedness
    const int hand_index = (snapshot.handedness == "left") ? ISAACSIM_HAND_LEFT_ : ISAACSIM_HAND_RIGHT_;
    const int other_hand_index = 1 - hand_index;

    // Calculate grip state from buttons (any button pressed = gripping)
    const bool gripping =
        snapshot.buttons.button_0 || snapshot.buttons.button_1 || snapshot.buttons.button_2 || snapshot.buttons.button_3;

    // Smooth the grip interpolant
    const float target_grip = gripping ? 1.0f : 0.0f;
    const float grip_speed = 0.15f; // Smoothing factor
    grip_interpolant_ += (target_grip - grip_interpolant_) * grip_speed;

    // Initialize all joints to zero/invalid for both hands
    for (int i = 0; i < required_count; ++i)
    {
        out_joint_poses[i].position[0] = 0.0f;
        out_joint_poses[i].position[1] = 0.0f;
        out_joint_poses[i].position[2] = 0.0f;
        out_joint_poses[i].orientation[0] = 0.0f;
        out_joint_poses[i].orientation[1] = 0.0f;
        out_joint_poses[i].orientation[2] = 0.0f;
        out_joint_poses[i].orientation[3] = 1.0f; // w=1 for identity
        out_joint_poses[i].radius = 0.01f;
        out_joint_poses[i].locationFlags = 0;
    }

    // Set WRIST joint - this has real tracking data
    const int wrist_idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
    out_joint_poses[wrist_idx].position[0] = snapshot.cursor_position.x;
    out_joint_poses[wrist_idx].position[1] = snapshot.cursor_position.y;
    out_joint_poses[wrist_idx].position[2] = snapshot.cursor_position.z;
    out_joint_poses[wrist_idx].orientation[0] = snapshot.orientation.x;
    out_joint_poses[wrist_idx].orientation[1] = snapshot.orientation.y;
    out_joint_poses[wrist_idx].orientation[2] = snapshot.orientation.z;
    out_joint_poses[wrist_idx].orientation[3] = snapshot.orientation.w;
    out_joint_poses[wrist_idx].radius = 0.03f;
    out_joint_poses[wrist_idx].locationFlags = ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID |
                                               ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID |
                                               ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_TRACKED |
                                               ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_TRACKED;

    // Set PALM joint - same as wrist for reference
    const int palm_idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_PALM;
    out_joint_poses[palm_idx] = out_joint_poses[wrist_idx];
    out_joint_poses[palm_idx].radius = 0.04f;

    // Synthesize finger joints based on grip interpolant
    // These are NOT tracked (locationFlags = 0) but provide visual feedback
    // Open hand: fingers extended; Closed hand: fingers curled

    // Define finger joint offsets relative to wrist in local (hand) space
    // Each finger has: metacarpal, proximal, intermediate (except thumb), distal, tip
    struct FingerDef
    {
        int base_joint;
        int joint_count;
        float lateral_offset; // Spread across palm
        float forward_base;   // Base distance forward from wrist
    };

    // Finger definitions (in order: thumb, index, middle, ring, little)
    const FingerDef fingers[] = {
        { ISAACSIM_HAND_JOINT_THUMB_METACARPAL, 4, -0.04f, 0.02f },
        { ISAACSIM_HAND_JOINT_INDEX_METACARPAL, 5, -0.02f, 0.04f },
        { ISAACSIM_HAND_JOINT_MIDDLE_METACARPAL, 5, 0.00f, 0.045f },
        { ISAACSIM_HAND_JOINT_RING_METACARPAL, 5, 0.02f, 0.04f },
        { ISAACSIM_HAND_JOINT_LITTLE_METACARPAL, 5, 0.04f, 0.035f },
    };

    // Segment lengths (approximate)
    const float segment_lengths[] = { 0.02f, 0.025f, 0.02f, 0.015f, 0.01f };

    const float& g = grip_interpolant_;
    const float curl_angle = g * 1.4f; // ~80 degrees when fully gripped

    for (const auto& finger : fingers)
    {
        for (int j = 0; j < finger.joint_count; ++j)
        {
            const int joint_enum = finger.base_joint + j;
            const int idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + joint_enum;

            // Compute position along the finger
            float forward = finger.forward_base;
            for (int k = 0; k < j; ++k)
            {
                // When curled, fingers bend downward
                forward += segment_lengths[k] * std::cos(curl_angle * static_cast<float>(k + 1) * 0.3f);
            }

            float down = 0.0f;
            for (int k = 0; k < j; ++k)
            {
                down -= segment_lengths[k] * std::sin(curl_angle * static_cast<float>(k + 1) * 0.3f);
            }

            // Transform to world space (simplified: assume wrist orientation is identity-ish)
            out_joint_poses[idx].position[0] = snapshot.cursor_position.x + forward;
            out_joint_poses[idx].position[1] = snapshot.cursor_position.y + finger.lateral_offset;
            out_joint_poses[idx].position[2] = snapshot.cursor_position.z + down;

            // Orientation: inherit from wrist (simplification)
            out_joint_poses[idx].orientation[0] = snapshot.orientation.x;
            out_joint_poses[idx].orientation[1] = snapshot.orientation.y;
            out_joint_poses[idx].orientation[2] = snapshot.orientation.z;
            out_joint_poses[idx].orientation[3] = snapshot.orientation.w;

            out_joint_poses[idx].radius = 0.008f;
            // Mark as valid but NOT tracked (synthesized data)
            out_joint_poses[idx].locationFlags =
                ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID | ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID;
        }
    }

    // The other hand has no data - already zeroed with locationFlags = 0
    (void)other_hand_index;

    return true;
}

} // namespace input_devices
} // namespace xr
} // namespace isaacsim


// ============================================================================
// Plugin registration bridge
// ============================================================================

namespace
{
using isaacsim::xr::input_devices::IsaacSimHaplyTracker;
static std::unique_ptr<IsaacSimHaplyTracker> g_tracker_instance;
static std::mutex g_tracker_mutex;
} // namespace

struct HaplyHandTrackingPlugin
{
    static bool Initialize()
    {
        std::lock_guard<std::mutex> lock(g_tracker_mutex);
        if (!g_tracker_instance)
        {
            g_tracker_instance = std::make_unique<IsaacSimHaplyTracker>();

            // Read WebSocket config from environment (optional)
            const char* host_env = std::getenv("HAPLY_WS_HOST");
            const char* port_env = std::getenv("HAPLY_WS_PORT");
            std::string host = host_env ? host_env : "127.0.0.1";
            uint16_t port = port_env ? static_cast<uint16_t>(std::atoi(port_env)) : 10001;

            if (!g_tracker_instance->initialize(host, port))
            {
                g_tracker_instance.reset();
                return false;
            }
        }
        return true;
    }

    static bool GetData(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count)
    {
        IsaacSimHaplyTracker* tracker = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_tracker_mutex);
            tracker = g_tracker_instance.get();
        }
        if (!tracker)
        {
            return false;
        }
        return tracker->get_hand_data(out_joint_poses, out_joint_pose_count);
    }

    static void Shutdown()
    {
        std::unique_ptr<IsaacSimHaplyTracker> to_cleanup;
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

// Register the implementation so the C API forwards to it.
ISAACSIM_HANDTRACKER_REGISTER(HaplyHandTrackingPlugin)
