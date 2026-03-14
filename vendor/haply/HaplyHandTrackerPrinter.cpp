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

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t g_should_exit = 0;

static void handle_signal(int)
{
    g_should_exit = 1;
}

static const char* joint_name(int idx)
{
    switch (idx)
    {
    case ISAACSIM_HAND_JOINT_WRIST:
        return "WRIST";
    case ISAACSIM_HAND_JOINT_THUMB_METACARPAL:
        return "THUMB_METACARPAL";
    case ISAACSIM_HAND_JOINT_THUMB_PROXIMAL:
        return "THUMB_PROXIMAL";
    case ISAACSIM_HAND_JOINT_THUMB_DISTAL:
        return "THUMB_DISTAL";
    case ISAACSIM_HAND_JOINT_THUMB_TIP:
        return "THUMB_TIP";
    case ISAACSIM_HAND_JOINT_INDEX_METACARPAL:
        return "INDEX_METACARPAL";
    case ISAACSIM_HAND_JOINT_INDEX_PROXIMAL:
        return "INDEX_PROXIMAL";
    case ISAACSIM_HAND_JOINT_INDEX_INTERMEDIATE:
        return "INDEX_INTERMEDIATE";
    case ISAACSIM_HAND_JOINT_INDEX_DISTAL:
        return "INDEX_DISTAL";
    case ISAACSIM_HAND_JOINT_INDEX_TIP:
        return "INDEX_TIP";
    case ISAACSIM_HAND_JOINT_MIDDLE_METACARPAL:
        return "MIDDLE_METACARPAL";
    case ISAACSIM_HAND_JOINT_MIDDLE_PROXIMAL:
        return "MIDDLE_PROXIMAL";
    case ISAACSIM_HAND_JOINT_MIDDLE_INTERMEDIATE:
        return "MIDDLE_INTERMEDIATE";
    case ISAACSIM_HAND_JOINT_MIDDLE_DISTAL:
        return "MIDDLE_DISTAL";
    case ISAACSIM_HAND_JOINT_MIDDLE_TIP:
        return "MIDDLE_TIP";
    case ISAACSIM_HAND_JOINT_RING_METACARPAL:
        return "RING_METACARPAL";
    case ISAACSIM_HAND_JOINT_RING_PROXIMAL:
        return "RING_PROXIMAL";
    case ISAACSIM_HAND_JOINT_RING_INTERMEDIATE:
        return "RING_INTERMEDIATE";
    case ISAACSIM_HAND_JOINT_RING_DISTAL:
        return "RING_DISTAL";
    case ISAACSIM_HAND_JOINT_RING_TIP:
        return "RING_TIP";
    case ISAACSIM_HAND_JOINT_LITTLE_METACARPAL:
        return "LITTLE_METACARPAL";
    case ISAACSIM_HAND_JOINT_LITTLE_PROXIMAL:
        return "LITTLE_PROXIMAL";
    case ISAACSIM_HAND_JOINT_LITTLE_INTERMEDIATE:
        return "LITTLE_INTERMEDIATE";
    case ISAACSIM_HAND_JOINT_LITTLE_DISTAL:
        return "LITTLE_DISTAL";
    case ISAACSIM_HAND_JOINT_LITTLE_TIP:
        return "LITTLE_TIP";
    case ISAACSIM_HAND_JOINT_PALM:
        return "PALM";
    default:
        return "UNKNOWN";
    }
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::cout << "=== Haply Hand Tracker Printer ===" << std::endl;
    std::cout << "Haply Inverse3 + VerseGrip -> Isaac Sim Hand Tracking C API" << std::endl;
    std::cout << std::endl;
    std::cout << "Environment variables (optional):" << std::endl;
    std::cout << "  HAPLY_WS_HOST - WebSocket host (default: 127.0.0.1)" << std::endl;
    std::cout << "  HAPLY_WS_PORT - WebSocket port (default: 10001)" << std::endl;
    std::cout << std::endl;

    std::cout << "Initializing IsaacSim Hand Tracker (Haply plugin)..." << std::endl;
    if (!IsaacSimHandTracker_Initialize())
    {
        std::cerr << "Failed to initialize hand tracker. Ensure the Haply SDK service is running." << std::endl;
        return 1;
    }

    const int pose_count = ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT;
    IsaacSimHandJointPose* buffer = new IsaacSimHandJointPose[pose_count];

    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    int frame = 0;
    while (!g_should_exit)
    {
        if (!IsaacSimHandTracker_GetData(buffer, pose_count))
        {
            std::cout << "Waiting for Haply device data..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        // Clear screen and print header
        std::cout << "\033[2J\033[H"; // ANSI clear screen
        std::cout << "=== Haply Hand Tracker - Frame " << frame << " ===" << std::endl;
        std::cout << std::endl;

        // Print Haply-specific summary (WRIST data = Inverse3 position + VerseGrip orientation)
        std::cout << "--- Haply Device Summary ---" << std::endl;

        // Find which hand has valid WRIST data
        for (int hand = 0; hand < ISAACSIM_HAND_COUNT; ++hand)
        {
            const int wrist_idx = hand * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
            const auto& wrist = buffer[wrist_idx];
            const bool tracked =
                (wrist.locationFlags & ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_TRACKED) != 0;

            if (tracked)
            {
                const char* hand_name = (hand == 0) ? "LEFT" : "RIGHT";
                std::cout << hand_name << " HAND (Inverse3 + VerseGrip):" << std::endl;
                std::cout << "  Position:    [" << std::fixed << std::setprecision(4)
                          << std::setw(8) << wrist.position[0] << ", "
                          << std::setw(8) << wrist.position[1] << ", "
                          << std::setw(8) << wrist.position[2] << "] m" << std::endl;
                std::cout << "  Orientation: [" << std::fixed << std::setprecision(4)
                          << std::setw(8) << wrist.orientation[0] << ", "
                          << std::setw(8) << wrist.orientation[1] << ", "
                          << std::setw(8) << wrist.orientation[2] << ", "
                          << std::setw(8) << wrist.orientation[3] << "] (xyzw)" << std::endl;
                std::cout << std::endl;
            }
        }

        std::cout << "--- Joint Poses (tracked joints marked with [T]) ---" << std::endl;

        auto print_joint = [&](int hand_index, int joint_index)
        {
            const int idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + joint_index;
            const auto& p = buffer[idx];
            const bool pos_valid = (p.locationFlags & ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID) != 0;
            const bool pos_tracked = (p.locationFlags & ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_TRACKED) != 0;

            if (!pos_valid)
            {
                return; // Skip joints with no data
            }

            const char* tracked_marker = pos_tracked ? "[T]" : "   ";
            std::cout << (hand_index == 0 ? "L" : "R") << " " << tracked_marker << " "
                      << std::left << std::setw(22) << joint_name(joint_index)
                      << " pos=[" << std::fixed << std::setprecision(3)
                      << std::setw(7) << p.position[0] << ", "
                      << std::setw(7) << p.position[1] << ", "
                      << std::setw(7) << p.position[2] << "]" << std::endl;
        };

        for (int hand = 0; hand < ISAACSIM_HAND_COUNT; ++hand)
        {
            for (int joint = 0; joint < ISAACSIM_HAND_JOINT_COUNT; ++joint)
            {
                print_joint(hand, joint);
            }
        }

        std::cout << std::endl;
        std::cout << "Legend: [T] = Tracked (real sensor data), others = Synthesized from grip state" << std::endl;
        std::cout << std::flush;

        frame++;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    delete[] buffer;
    IsaacSimHandTracker_Shutdown();
    std::cout << std::endl << "Shutdown complete." << std::endl;
    return 0;
}
