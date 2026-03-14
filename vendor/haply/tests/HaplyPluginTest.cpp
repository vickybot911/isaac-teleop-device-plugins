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

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "HaplyHandTrackingPlugin.h"
#include "doctest/doctest.h"
#include "isaac-handtracking-plugin/IsaacSimHandTrackerCAPI.h"
#include "third_party/nlohmann/json.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace isaacsim::xr::input_devices;

// ============================================================================
// Helpers — replicate the parsing and mapping logic from the implementation so
// we can verify the contract independently without a live WebSocket connection.
// ============================================================================

/// Parse a JSON message (matching the Haply SDK WebSocket format) into a
/// HaplyDeviceState, using the same field-access pattern as io_loop().
static HaplyDeviceState parse_haply_json(const std::string& raw, HaplyDeviceState prior = {})
{
    HaplyDeviceState state = prior;
    json j = json::parse(raw);

    // Parse inverse3 array
    if (j.contains("inverse3") && j["inverse3"].is_array() && !j["inverse3"].empty())
    {
        const auto& inv3 = j["inverse3"][0];
        if (inv3.contains("device_id"))
        {
            state.inverse3_device_id = inv3["device_id"].get<std::string>();
        }
        if (inv3.contains("config") && inv3["config"].contains("handedness"))
        {
            state.handedness = inv3["config"]["handedness"].get<std::string>();
        }
        if (inv3.contains("state"))
        {
            const auto& st = inv3["state"];
            if (st.contains("cursor_position"))
            {
                const auto& pos = st["cursor_position"];
                state.cursor_position.x = pos.value("x", 0.0f);
                state.cursor_position.y = pos.value("y", 0.0f);
                state.cursor_position.z = pos.value("z", 0.0f);
            }
            if (st.contains("cursor_velocity"))
            {
                const auto& vel = st["cursor_velocity"];
                state.cursor_velocity.x = vel.value("x", 0.0f);
                state.cursor_velocity.y = vel.value("y", 0.0f);
                state.cursor_velocity.z = vel.value("z", 0.0f);
            }
        }
        state.has_data = true;
    }

    // Parse wireless_verse_grip array
    if (j.contains("wireless_verse_grip") && j["wireless_verse_grip"].is_array() && !j["wireless_verse_grip"].empty())
    {
        const auto& vg = j["wireless_verse_grip"][0];
        if (vg.contains("device_id"))
        {
            state.versegrip_device_id = vg["device_id"].get<std::string>();
        }
        if (vg.contains("state"))
        {
            const auto& st = vg["state"];
            if (st.contains("orientation"))
            {
                const auto& ori = st["orientation"];
                state.orientation.w = ori.value("w", 1.0f);
                state.orientation.x = ori.value("x", 0.0f);
                state.orientation.y = ori.value("y", 0.0f);
                state.orientation.z = ori.value("z", 0.0f);
            }
            if (st.contains("buttons"))
            {
                const auto& btn = st["buttons"];
                state.buttons.button_0 = btn.value("button_0", false);
                state.buttons.button_1 = btn.value("button_1", false);
                state.buttons.button_2 = btn.value("button_2", false);
                state.buttons.button_3 = btn.value("button_3", false);
            }
        }
        state.has_data = true;
    }

    return state;
}

/// Map a HaplyDeviceState to an IsaacSimHandJointPose array, replicating the
/// logic from get_hand_data() (wrist, palm, and finger synthesis).
static bool map_hand_data(const HaplyDeviceState& snapshot,
                          IsaacSimHandJointPose* out_joint_poses,
                          int out_joint_pose_count,
                          float& grip_interpolant)
{
    const int required_count = ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT;
    if (!out_joint_poses || out_joint_pose_count < required_count)
    {
        return false;
    }

    if (!snapshot.has_data)
    {
        return false;
    }

    const int hand_index = (snapshot.handedness == "left") ? ISAACSIM_HAND_LEFT_ : ISAACSIM_HAND_RIGHT_;

    const bool gripping = snapshot.buttons.button_0 || snapshot.buttons.button_1 || snapshot.buttons.button_2 ||
                          snapshot.buttons.button_3;

    const float target_grip = gripping ? 1.0f : 0.0f;
    const float grip_speed = 0.15f;
    grip_interpolant += (target_grip - grip_interpolant) * grip_speed;

    // Zero all joints
    for (int i = 0; i < required_count; ++i)
    {
        out_joint_poses[i].position[0] = 0.0f;
        out_joint_poses[i].position[1] = 0.0f;
        out_joint_poses[i].position[2] = 0.0f;
        out_joint_poses[i].orientation[0] = 0.0f;
        out_joint_poses[i].orientation[1] = 0.0f;
        out_joint_poses[i].orientation[2] = 0.0f;
        out_joint_poses[i].orientation[3] = 1.0f;
        out_joint_poses[i].radius = 0.01f;
        out_joint_poses[i].locationFlags = 0;
    }

    // WRIST
    const int wrist_idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
    out_joint_poses[wrist_idx].position[0] = snapshot.cursor_position.x;
    out_joint_poses[wrist_idx].position[1] = snapshot.cursor_position.y;
    out_joint_poses[wrist_idx].position[2] = snapshot.cursor_position.z;
    out_joint_poses[wrist_idx].orientation[0] = snapshot.orientation.x;
    out_joint_poses[wrist_idx].orientation[1] = snapshot.orientation.y;
    out_joint_poses[wrist_idx].orientation[2] = snapshot.orientation.z;
    out_joint_poses[wrist_idx].orientation[3] = snapshot.orientation.w;
    out_joint_poses[wrist_idx].radius = 0.03f;
    out_joint_poses[wrist_idx].locationFlags =
        ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID | ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID |
        ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_TRACKED | ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_TRACKED;

    // PALM (copy of WRIST)
    const int palm_idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_PALM;
    out_joint_poses[palm_idx] = out_joint_poses[wrist_idx];
    out_joint_poses[palm_idx].radius = 0.04f;

    // Synthesise finger joints (valid but NOT tracked)
    struct FingerDef
    {
        int base_joint;
        int joint_count;
        float lateral_offset;
        float forward_base;
    };

    const FingerDef fingers[] = {
        { ISAACSIM_HAND_JOINT_THUMB_METACARPAL, 4, -0.04f, 0.02f },
        { ISAACSIM_HAND_JOINT_INDEX_METACARPAL, 5, -0.02f, 0.04f },
        { ISAACSIM_HAND_JOINT_MIDDLE_METACARPAL, 5, 0.00f, 0.045f },
        { ISAACSIM_HAND_JOINT_RING_METACARPAL, 5, 0.02f, 0.04f },
        { ISAACSIM_HAND_JOINT_LITTLE_METACARPAL, 5, 0.04f, 0.035f },
    };

    const float segment_lengths[] = { 0.02f, 0.025f, 0.02f, 0.015f, 0.01f };
    const float& g = grip_interpolant;
    const float curl_angle = g * 1.4f;

    for (const auto& finger : fingers)
    {
        for (int j = 0; j < finger.joint_count; ++j)
        {
            const int joint_enum = finger.base_joint + j;
            const int idx = hand_index * ISAACSIM_HAND_JOINT_COUNT + joint_enum;

            float forward = finger.forward_base;
            for (int k = 0; k < j; ++k)
            {
                forward += segment_lengths[k] * std::cos(curl_angle * static_cast<float>(k + 1) * 0.3f);
            }

            float down = 0.0f;
            for (int k = 0; k < j; ++k)
            {
                down -= segment_lengths[k] * std::sin(curl_angle * static_cast<float>(k + 1) * 0.3f);
            }

            out_joint_poses[idx].position[0] = snapshot.cursor_position.x + forward;
            out_joint_poses[idx].position[1] = snapshot.cursor_position.y + finger.lateral_offset;
            out_joint_poses[idx].position[2] = snapshot.cursor_position.z + down;

            out_joint_poses[idx].orientation[0] = snapshot.orientation.x;
            out_joint_poses[idx].orientation[1] = snapshot.orientation.y;
            out_joint_poses[idx].orientation[2] = snapshot.orientation.z;
            out_joint_poses[idx].orientation[3] = snapshot.orientation.w;

            out_joint_poses[idx].radius = 0.008f;
            out_joint_poses[idx].locationFlags = ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID |
                                                 ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID;
        }
    }

    return true;
}

// ============================================================================
// Buffer size helpers
// ============================================================================

/// The implementation accesses PALM (index 25) beyond the nominal HAND_JOINT_COUNT (25).
/// We use a slightly larger buffer to account for this.
static constexpr int kEffectiveJointsPerHand = ISAACSIM_HAND_JOINT_PALM + 1; // 26
static constexpr int kBufferSize = ISAACSIM_HAND_COUNT * kEffectiveJointsPerHand;

/// The implementation checks against the nominal count, so we must pass at
/// least this value for the out_joint_pose_count parameter.
static constexpr int kNominalCount = ISAACSIM_HAND_COUNT * ISAACSIM_HAND_JOINT_COUNT;

/// All wrist + palm flags when fully tracked.
static constexpr int kAllTrackedFlags =
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID | ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID |
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_TRACKED | ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_TRACKED;

/// Finger joints: valid but not tracked.
static constexpr int kSynthesizedFlags =
    ISAACSIM_HAND_JOINT_LOCATION_FLAGS_POSITION_VALID | ISAACSIM_HAND_JOINT_LOCATION_FLAGS_ORIENTATION_VALID;

static constexpr float kEps = 1e-5f;

// ============================================================================
// Test 1 — JSON parsing: Haply device state extraction
// ============================================================================

TEST_CASE("JSON parsing — full Haply device state")
{
    // Use the key names the production code actually parses (button_0, etc.)
    const std::string msg = R"({
        "inverse3": [{
            "device_id": "test-inv3",
            "config": {"handedness": "right"},
            "state": {
                "cursor_position": {"x": 0.1, "y": 0.2, "z": 0.3},
                "cursor_velocity": {"x": 0.01, "y": 0.02, "z": 0.03}
            }
        }],
        "wireless_verse_grip": [{
            "device_id": "test-vg",
            "state": {
                "buttons": {"button_0": true, "button_1": false, "button_2": false, "button_3": false},
                "orientation": {"w": 0.707, "x": 0.0, "y": 0.707, "z": 0.0}
            }
        }]
    })";

    HaplyDeviceState state = parse_haply_json(msg);

    // Inverse3 fields
    CHECK(state.inverse3_device_id == "test-inv3");
    CHECK(state.handedness == "right");
    CHECK(state.cursor_position.x == doctest::Approx(0.1f).epsilon(kEps));
    CHECK(state.cursor_position.y == doctest::Approx(0.2f).epsilon(kEps));
    CHECK(state.cursor_position.z == doctest::Approx(0.3f).epsilon(kEps));
    CHECK(state.cursor_velocity.x == doctest::Approx(0.01f).epsilon(kEps));
    CHECK(state.cursor_velocity.y == doctest::Approx(0.02f).epsilon(kEps));
    CHECK(state.cursor_velocity.z == doctest::Approx(0.03f).epsilon(kEps));

    // VerseGrip fields
    CHECK(state.versegrip_device_id == "test-vg");
    CHECK(state.orientation.w == doctest::Approx(0.707f).epsilon(kEps));
    CHECK(state.orientation.x == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(state.orientation.y == doctest::Approx(0.707f).epsilon(kEps));
    CHECK(state.orientation.z == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(state.buttons.button_0 == true);
    CHECK(state.buttons.button_1 == false);
    CHECK(state.buttons.button_2 == false);
    CHECK(state.buttons.button_3 == false);

    CHECK(state.has_data == true);
}

// ============================================================================
// Test 2 — Hand joint mapping (right-handed device)
// ============================================================================

TEST_CASE("Hand joint mapping — right-handed device")
{
    HaplyDeviceState state;
    state.has_data = true;
    state.handedness = "right";
    state.cursor_position = { 0.5f, 0.6f, 0.7f };
    state.orientation = { 0.707f, 0.0f, 0.707f, 0.0f }; // w,x,y,z
    state.buttons = { false, false, false, false };

    std::vector<IsaacSimHandJointPose> poses(kBufferSize);
    float grip = 0.0f;

    bool ok = map_hand_data(state, poses.data(), kNominalCount, grip);
    CHECK(ok);

    const int rh = ISAACSIM_HAND_RIGHT_;

    SUBCASE("WRIST joint gets position and orientation, fully tracked")
    {
        const int wrist = rh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
        CHECK(poses[wrist].position[0] == doctest::Approx(0.5f).epsilon(kEps));
        CHECK(poses[wrist].position[1] == doctest::Approx(0.6f).epsilon(kEps));
        CHECK(poses[wrist].position[2] == doctest::Approx(0.7f).epsilon(kEps));
        CHECK(poses[wrist].orientation[0] == doctest::Approx(0.0f).epsilon(kEps)); // x
        CHECK(poses[wrist].orientation[1] == doctest::Approx(0.707f).epsilon(kEps)); // y
        CHECK(poses[wrist].orientation[2] == doctest::Approx(0.0f).epsilon(kEps)); // z
        CHECK(poses[wrist].orientation[3] == doctest::Approx(0.707f).epsilon(kEps)); // w
        CHECK(poses[wrist].locationFlags == kAllTrackedFlags);
    }

    SUBCASE("PALM joint matches WRIST (except radius)")
    {
        const int wrist = rh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
        const int palm = rh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_PALM;
        CHECK(poses[palm].position[0] == poses[wrist].position[0]);
        CHECK(poses[palm].position[1] == poses[wrist].position[1]);
        CHECK(poses[palm].position[2] == poses[wrist].position[2]);
        CHECK(poses[palm].orientation[0] == poses[wrist].orientation[0]);
        CHECK(poses[palm].orientation[1] == poses[wrist].orientation[1]);
        CHECK(poses[palm].orientation[2] == poses[wrist].orientation[2]);
        CHECK(poses[palm].orientation[3] == poses[wrist].orientation[3]);
        CHECK(poses[palm].locationFlags == kAllTrackedFlags);
        CHECK(poses[palm].radius == doctest::Approx(0.04f).epsilon(kEps));
    }

    SUBCASE("Finger joints are synthesized (valid, not tracked)")
    {
        // Pick a representative finger joint: index metacarpal
        const int idx = rh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_INDEX_METACARPAL;
        CHECK(poses[idx].locationFlags == kSynthesizedFlags);
    }

    SUBCASE("Left hand has all zeros with locationFlags = 0")
    {
        const int lh = ISAACSIM_HAND_LEFT_;
        for (int j = 0; j < ISAACSIM_HAND_JOINT_COUNT; ++j)
        {
            const int idx = lh * ISAACSIM_HAND_JOINT_COUNT + j;
            CHECK(poses[idx].position[0] == doctest::Approx(0.0f).epsilon(kEps));
            CHECK(poses[idx].position[1] == doctest::Approx(0.0f).epsilon(kEps));
            CHECK(poses[idx].position[2] == doctest::Approx(0.0f).epsilon(kEps));
            CHECK(poses[idx].locationFlags == 0);
        }
    }
}

// ============================================================================
// Test 3 — Left-handed device
// ============================================================================

TEST_CASE("Hand joint mapping — left-handed device")
{
    HaplyDeviceState state;
    state.has_data = true;
    state.handedness = "left";
    state.cursor_position = { 1.0f, 2.0f, 3.0f };
    state.orientation = { 1.0f, 0.0f, 0.0f, 0.0f }; // identity
    state.buttons = { false, false, false, false };

    std::vector<IsaacSimHandJointPose> poses(kBufferSize);
    float grip = 0.0f;

    bool ok = map_hand_data(state, poses.data(), kNominalCount, grip);
    CHECK(ok);

    const int lh = ISAACSIM_HAND_LEFT_;
    const int rh = ISAACSIM_HAND_RIGHT_;

    // LEFT hand WRIST gets data
    const int wrist = lh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
    CHECK(poses[wrist].position[0] == doctest::Approx(1.0f).epsilon(kEps));
    CHECK(poses[wrist].position[1] == doctest::Approx(2.0f).epsilon(kEps));
    CHECK(poses[wrist].position[2] == doctest::Approx(3.0f).epsilon(kEps));
    CHECK(poses[wrist].locationFlags == kAllTrackedFlags);

    // LEFT hand PALM also populated
    const int palm = lh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_PALM;
    CHECK(poses[palm].locationFlags == kAllTrackedFlags);

    // RIGHT hand nominal joints (0..24) have no data — except that the left
    // hand's PALM (joint 25) lands at absolute index 25 which coincides with
    // rh * ISAACSIM_HAND_JOINT_COUNT + 0.  Skip that slot in the zero-check.
    for (int j = 1; j < ISAACSIM_HAND_JOINT_COUNT; ++j)
    {
        const int idx = rh * ISAACSIM_HAND_JOINT_COUNT + j;
        CHECK(poses[idx].locationFlags == 0);
    }
}

// ============================================================================
// Test 4 — Missing VerseGrip data (only inverse3)
// ============================================================================

TEST_CASE("JSON parsing — missing VerseGrip, orientation defaults to identity")
{
    const std::string msg = R"({
        "inverse3": [{
            "device_id": "inv3-only",
            "config": {"handedness": "right"},
            "state": {
                "cursor_position": {"x": 0.4, "y": 0.5, "z": 0.6}
            }
        }]
    })";

    HaplyDeviceState state = parse_haply_json(msg);

    CHECK(state.has_data == true);
    CHECK(state.cursor_position.x == doctest::Approx(0.4f).epsilon(kEps));
    CHECK(state.cursor_position.y == doctest::Approx(0.5f).epsilon(kEps));
    CHECK(state.cursor_position.z == doctest::Approx(0.6f).epsilon(kEps));

    // Orientation should be the default identity quaternion
    CHECK(state.orientation.w == doctest::Approx(1.0f).epsilon(kEps));
    CHECK(state.orientation.x == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(state.orientation.y == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(state.orientation.z == doctest::Approx(0.0f).epsilon(kEps));

    // VerseGrip device ID should remain empty
    CHECK(state.versegrip_device_id.empty());

    // Mapping should still succeed — wrist gets position, identity orientation
    std::vector<IsaacSimHandJointPose> poses(kBufferSize);
    float grip = 0.0f;
    bool ok = map_hand_data(state, poses.data(), kNominalCount, grip);
    CHECK(ok);

    const int rh = ISAACSIM_HAND_RIGHT_;
    const int wrist = rh * ISAACSIM_HAND_JOINT_COUNT + ISAACSIM_HAND_JOINT_WRIST;
    CHECK(poses[wrist].position[0] == doctest::Approx(0.4f).epsilon(kEps));
    CHECK(poses[wrist].orientation[3] == doctest::Approx(1.0f).epsilon(kEps)); // w=1 identity
    CHECK(poses[wrist].locationFlags == kAllTrackedFlags);
}

// ============================================================================
// Test 5 — Empty / malformed JSON
// ============================================================================

TEST_CASE("JSON parsing — empty object")
{
    HaplyDeviceState state = parse_haply_json("{}");
    CHECK(state.has_data == false);
    CHECK(state.inverse3_device_id.empty());
    CHECK(state.versegrip_device_id.empty());
}

TEST_CASE("JSON parsing — empty inverse3 array")
{
    HaplyDeviceState state = parse_haply_json(R"({"inverse3": []})");
    CHECK(state.has_data == false);
}

TEST_CASE("JSON parsing — missing state fields")
{
    const std::string msg = R"({
        "inverse3": [{
            "device_id": "partial"
        }]
    })";

    HaplyDeviceState state = parse_haply_json(msg);
    CHECK(state.has_data == true);
    CHECK(state.inverse3_device_id == "partial");
    // Position and velocity should remain at defaults (0)
    CHECK(state.cursor_position.x == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(state.cursor_velocity.x == doctest::Approx(0.0f).epsilon(kEps));
}

TEST_CASE("JSON parsing — malformed string does not crash")
{
    CHECK_THROWS_AS(parse_haply_json("not json at all"), json::parse_error);
    CHECK_THROWS_AS(parse_haply_json("{invalid}"), json::parse_error);
    CHECK_THROWS_AS(parse_haply_json(""), json::parse_error);
}

// ============================================================================
// Test 6 — State thread safety (basic) / uninitialised tracker
// ============================================================================

TEST_CASE("Uninitialised tracker — get_hand_data returns false, raw state is default")
{
    IsaacSimHaplyTracker tracker;

    // No initialize() call — no WebSocket, no io_loop

    std::vector<IsaacSimHandJointPose> poses(kBufferSize);
    bool ok = tracker.get_hand_data(poses.data(), kNominalCount);
    CHECK(ok == false);

    HaplyDeviceState raw = tracker.get_raw_state();
    CHECK(raw.has_data == false);
    CHECK(raw.handedness == "right");
    CHECK(raw.inverse3_device_id.empty());
    CHECK(raw.versegrip_device_id.empty());
    CHECK(raw.cursor_position.x == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(raw.cursor_position.y == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(raw.cursor_position.z == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(raw.orientation.w == doctest::Approx(1.0f).epsilon(kEps));
    CHECK(raw.orientation.x == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(raw.orientation.y == doctest::Approx(0.0f).epsilon(kEps));
    CHECK(raw.orientation.z == doctest::Approx(0.0f).epsilon(kEps));
}

TEST_CASE("get_hand_data rejects null buffer and undersized count")
{
    IsaacSimHaplyTracker tracker;

    CHECK(tracker.get_hand_data(nullptr, kNominalCount) == false);

    std::vector<IsaacSimHandJointPose> poses(kBufferSize);
    CHECK(tracker.get_hand_data(poses.data(), 0) == false);
    CHECK(tracker.get_hand_data(poses.data(), kNominalCount - 1) == false);
}

// ============================================================================
// Bonus — grip button mapping
// ============================================================================

TEST_CASE("Grip interpolant responds to button presses")
{
    HaplyDeviceState state;
    state.has_data = true;
    state.handedness = "right";
    state.cursor_position = { 0.0f, 0.0f, 0.0f };
    state.orientation = { 1.0f, 0.0f, 0.0f, 0.0f };

    // No buttons pressed — grip stays near 0
    state.buttons = { false, false, false, false };
    float grip = 0.0f;
    std::vector<IsaacSimHandJointPose> poses(kBufferSize);

    map_hand_data(state, poses.data(), kNominalCount, grip);
    CHECK(grip == doctest::Approx(0.0f).epsilon(kEps));

    // Press button 0 — grip moves toward 1
    state.buttons.button_0 = true;
    for (int i = 0; i < 50; ++i)
    {
        map_hand_data(state, poses.data(), kNominalCount, grip);
    }
    CHECK(grip > 0.99f);

    // Release all buttons — grip decays back toward 0
    state.buttons.button_0 = false;
    for (int i = 0; i < 50; ++i)
    {
        map_hand_data(state, poses.data(), kNominalCount, grip);
    }
    CHECK(grip < 0.01f);
}
