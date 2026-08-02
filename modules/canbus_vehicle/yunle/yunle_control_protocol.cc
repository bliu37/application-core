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

#include "modules/canbus_vehicle/yunle/yunle_control_protocol.h"

#include <cmath>
#include <limits>

namespace apollo {
namespace canbus {
namespace yunle {

namespace {

constexpr double kMaximumTargetSpeedKph = 51.1;
constexpr double kMaximumSteeringPercentage = 100.0;
constexpr int32_t kMaximumSteeringCount = 120;
constexpr double kSpeedQuantizationEpsilon = 1e-6;

bool IsValidDriveMode(YunleDriveModeRequest mode) {
  return mode == YunleDriveModeRequest::kAutonomous ||
         mode == YunleDriveModeRequest::kRemote;
}

bool IsValidGear(YunleGearRequest gear) {
  const uint8_t raw = static_cast<uint8_t>(gear);
  return raw <= static_cast<uint8_t>(YunleGearRequest::kReverse);
}

bool IsValidTwoBitRequest(uint8_t request) { return request <= 3U; }

bool EncodeSteeringPercentage(double percentage, uint8_t* encoded) {
  if (encoded == nullptr || !std::isfinite(percentage) ||
      percentage < -kMaximumSteeringPercentage ||
      percentage > kMaximumSteeringPercentage) {
    return false;
  }

  // JD03 uses signed 8-bit counts with left negative and right positive.
  const int32_t count = static_cast<int32_t>(std::lround(
      -percentage * kMaximumSteeringCount / kMaximumSteeringPercentage));
  if (count < -kMaximumSteeringCount || count > kMaximumSteeringCount) {
    return false;
  }
  *encoded = static_cast<uint8_t>(static_cast<int8_t>(count));
  return true;
}

}  // namespace

bool EncodeScuControl121(
    const YunleScuControlCommand& command,
    std::array<uint8_t, kScuControlLength>* data) {
  if (data == nullptr || !IsValidDriveMode(command.drive_mode) ||
      !IsValidGear(command.gear) ||
      !std::isfinite(command.target_speed_kph) ||
      command.target_speed_kph < 0.0 ||
      command.target_speed_kph > kMaximumTargetSpeedKph ||
      !IsValidTwoBitRequest(command.left_turn_light_request) ||
      !IsValidTwoBitRequest(command.right_turn_light_request) ||
      !IsValidTwoBitRequest(command.position_light_request) ||
      !IsValidTwoBitRequest(command.low_beam_request)) {
    return false;
  }

  uint8_t front_steering = 0U;
  uint8_t rear_steering = 0U;
  if (!EncodeSteeringPercentage(command.front_steering_percentage,
                                &front_steering) ||
      !EncodeSteeringPercentage(command.rear_steering_percentage,
                                &rear_steering)) {
    return false;
  }

  // JD03 represents speed in 0.1 km/h steps. Truncate non-negative requests
  // so wire-level quantization can never command a speed above the request.
  const uint32_t target_speed = static_cast<uint32_t>(
      std::floor(command.target_speed_kph * 10.0 +
                 kSpeedQuantizationEpsilon));
  if (target_speed > std::numeric_limits<uint16_t>::max() ||
      target_speed > 0x1FFU) {
    return false;
  }

  data->fill(0U);
  (*data)[0] = static_cast<uint8_t>(
      (static_cast<uint8_t>(command.drive_mode) << 6U) |
      static_cast<uint8_t>(command.gear));
  (*data)[1] = front_steering;
  (*data)[2] = rear_steering;
  (*data)[3] = static_cast<uint8_t>(target_speed & 0xFFU);
  (*data)[4] = static_cast<uint8_t>(
      ((target_speed >> 8U) & 0x01U) |
      (static_cast<uint8_t>(command.brake_enable) << 1U));
  (*data)[5] = static_cast<uint8_t>(
      command.left_turn_light_request |
      (command.right_turn_light_request << 2U) |
      (command.position_light_request << 6U));
  (*data)[6] = command.low_beam_request;
  return true;
}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
