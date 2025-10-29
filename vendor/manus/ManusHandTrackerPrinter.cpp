// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

    std::cout << "Initializing IsaacSim Hand Tracker..." << std::endl;
    if (!IsaacSimHandTracker_Initialize())
    {
        std::cerr << "Failed to initialize hand tracker. Ensure the Manus plugin is available." << std::endl;
        return 1;
    }

    const int pose_count = ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT;
    IsaacSimHandJointPose* buffer = new IsaacSimHandJointPose[pose_count];

    std::cout << "Press Ctrl+C to stop. Printing ALL joints for both hands." << std::endl;

    int frame = 0;
    while (!g_should_exit)
    {
        if (!IsaacSimHandTracker_GetData(buffer, pose_count))
        {
            std::cout << "No data available yet..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        auto print_joint = [&](int hand_index, int joint_index)
        {
            const int idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + joint_index;
            const auto& p = buffer[idx];
            const bool pos_valid = (p.locationFlags & ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID) != 0;
            const bool ori_valid = (p.locationFlags & ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID) != 0;
            std::cout << (hand_index == 0 ? "LEFT" : "RIGHT") << ": " << joint_name(joint_index)
                      << " pos=" << (pos_valid ? "[" : "(") << std::fixed << std::setprecision(3) << p.position[0]
                      << ", " << p.position[1] << ", " << p.position[2] << (pos_valid ? "]" : ")")
                      << " ori=" << (ori_valid ? "[" : "(") << p.orientation[0] << ", " << p.orientation[1] << ", "
                      << p.orientation[2] << ", " << p.orientation[3] << (ori_valid ? "]" : ")") << " r=" << p.radius
                      << std::endl;
        };

        std::cout << "Frame " << frame << std::endl;
        for (int hand = 0; hand < ISAACSIM_HAND_COUNT; ++hand)
        {
            for (int joint = 0; joint < ISAACSIM_HAND_JOINT_COUNT; ++joint)
            {
                print_joint(hand, joint);
            }
        }
        std::cout << std::flush;

        frame++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    delete[] buffer;
    IsaacSimHandTracker_Shutdown();
    std::cout << "Shutdown complete." << std::endl;
    return 0;
}
