/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace apollo {
namespace canbus {
namespace yunle {

constexpr uint32_t kScuControlId = 0x121U;
constexpr size_t kScuControlLength = 8U;

enum class YunleDriveModeRequest : uint8_t {
  kAutonomous = 1U,
  kRemote = 3U,
};

enum class YunleGearRequest : uint8_t {
  kInvalid = 0U,
  kDrive = 1U,
  kNeutral = 2U,
  kReverse = 3U,
};

struct YunleScuControlCommand {
  YunleDriveModeRequest drive_mode = YunleDriveModeRequest::kRemote;
  YunleGearRequest gear = YunleGearRequest::kInvalid;

  // Non-negative speed magnitude. Direction is selected by gear.
  double target_speed_kph = 0.0;

  // Apollo convention: left is positive and right is negative. Each value is
  // a percentage of Yunle's full-scale command (120 protocol counts).
  double front_steering_percentage = 0.0;
  double rear_steering_percentage = 0.0;

  bool brake_enable = false;
  uint8_t left_turn_light_request = 0U;
  uint8_t right_turn_light_request = 0U;
  uint8_t position_light_request = 0U;
  uint8_t low_beam_request = 0U;
};

// Encodes only the eight CAN data bytes for JD03 CAN ID 0x121. This pure
// function performs no I/O and cannot transmit a CAN or UDP frame.
bool EncodeScuControl121(
    const YunleScuControlCommand& command,
    std::array<uint8_t, kScuControlLength>* data);

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
