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
#ifndef ISAACSIM_HAND_TRACKER_CAPI_H
#define ISAACSIM_HAND_TRACKER_CAPI_H

#ifdef __cplusplus
#    include <cstdbool>
#else
#    include <stdbool.h>
#endif

typedef enum IsaacSimHand
{
    ISAACSIM_HAND_LEFT_ = 0,
    ISAACSIM_HAND_RIGHT_ = 1,
} IsaacSimHand;

typedef enum IsaacSimHandJoints
{
    ISAACSIM_HAND_JOINT_WRIST = 0,
    ISAACSIM_HAND_JOINT_THUMB_METACARPAL = 1,
    ISAACSIM_HAND_JOINT_THUMB_PROXIMAL = 2,
    ISAACSIM_HAND_JOINT_THUMB_DISTAL = 3,
    ISAACSIM_HAND_JOINT_THUMB_TIP = 4,
    ISAACSIM_HAND_JOINT_INDEX_METACARPAL = 5,
    ISAACSIM_HAND_JOINT_INDEX_PROXIMAL = 6,
    ISAACSIM_HAND_JOINT_INDEX_INTERMEDIATE = 7,
    ISAACSIM_HAND_JOINT_INDEX_DISTAL = 8,
    ISAACSIM_HAND_JOINT_INDEX_TIP = 9,
    ISAACSIM_HAND_JOINT_MIDDLE_METACARPAL = 10,
    ISAACSIM_HAND_JOINT_MIDDLE_PROXIMAL = 11,
    ISAACSIM_HAND_JOINT_MIDDLE_INTERMEDIATE = 12,
    ISAACSIM_HAND_JOINT_MIDDLE_DISTAL = 13,
    ISAACSIM_HAND_JOINT_MIDDLE_TIP = 14,
    ISAACSIM_HAND_JOINT_RING_METACARPAL = 15,
    ISAACSIM_HAND_JOINT_RING_PROXIMAL = 16,
    ISAACSIM_HAND_JOINT_RING_INTERMEDIATE = 17,
    ISAACSIM_HAND_JOINT_RING_DISTAL = 18,
    ISAACSIM_HAND_JOINT_RING_TIP = 19,
    ISAACSIM_HAND_JOINT_LITTLE_METACARPAL = 20,
    ISAACSIM_HAND_JOINT_LITTLE_PROXIMAL = 21,
    ISAACSIM_HAND_JOINT_LITTLE_INTERMEDIATE = 22,
    ISAACSIM_HAND_JOINT_LITTLE_DISTAL = 23,
    ISAACSIM_HAND_JOINT_LITTLE_TIP = 24,
    ISAACSIM_HAND_JOINT_PALM = 25,
    ISAACSIM_HAND_JOINT_MAX_ENUM = 0x7FFFFFFF
} IsaacSimHandJoints;

typedef enum IsaacSimHandJointLocationFlags
{
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID = 0x00000001,
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID = 0x00000002,
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_TRACKED = 0x00000004,
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_TRACKED = 0x00000008,
} IsaacSimHandJointLocationFlags;

typedef struct IsaacSimHandJointPose
{
    // [x, y, z]
    float position[3];
    // [x, y, z, w]
    float orientation[4];
    // in meters
    float radius;
    // bitfield of IsaacSimHandJointLocationFlags
    int locationFlags;
} IsaacSimHandJointPose;

#ifndef ISAACSIM_HAND_COUNT
#    define ISAACSIM_HAND_COUNT 2
#endif

#ifndef ISAACSIM_HAND_JOINT_COUNT
#    define ISAACSIM_HAND_JOINT_COUNT (ISAACSIM_HAND_JOINT_LITTLE_TIP + 1)
#endif

#ifdef _MSC_VER
#    ifdef ISAACSIM_HANDTRACKER_CAPI_EXPORTS
#        define ISAACSIM_HANDTRACKER_API __declspec(dllexport)
#    else
#        define ISAACSIM_HANDTRACKER_API __declspec(dllimport)
#    endif
#else
#    define ISAACSIM_HANDTRACKER_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    // Initializes the hand tracking device. Returns true on success.
    ISAACSIM_HANDTRACKER_API bool IsaacSimHandTracker_Initialize(void);

    // Retrieves the latest joint poses for both hands.
    // The caller must provide a buffer of size (ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT),
    // ordered first by hand (IsaacSimHand: LEFT, RIGHT), then by IsaacSimHandJoints.
    // Returns true if data was written successfully.
    ISAACSIM_HANDTRACKER_API bool IsaacSimHandTracker_GetData(IsaacSimHandJointPose* out_joint_poses,
                                                              int out_joint_pose_count);

    // Shuts down the hand tracking device and releases resources.
    ISAACSIM_HANDTRACKER_API void IsaacSimHandTracker_Shutdown(void);

    // Optional function pointer typedefs for dynamic loading via dlsym/GetProcAddress.
    typedef bool (*IsaacSimHandTracker_Initialize_Func)(void);
    typedef bool (*IsaacSimHandTracker_GetData_Func)(IsaacSimHandJointPose* out_joint_poses, int out_joint_pose_count);
    typedef void (*IsaacSimHandTracker_Shutdown_Func)(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // ISAACSIM_HAND_TRACKER_CAPI_H
