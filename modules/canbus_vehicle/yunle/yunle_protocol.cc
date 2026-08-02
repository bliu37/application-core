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

#include "modules/canbus_vehicle/yunle/yunle_protocol.h"

#include <algorithm>
#include <array>
#include <limits>

namespace apollo {
namespace canbus {
namespace yunle {

namespace {

uint64_t LoadLittleEndian(const uint8_t* data) {
  uint64_t raw = 0;
  for (size_t i = 0; i < 8; ++i) {
    raw |= static_cast<uint64_t>(data[i]) << (i * 8U);
  }
  return raw;
}

uint64_t ExtractUnsigned(const uint8_t* data, uint32_t start_bit,
                         uint32_t bit_length) {
  const uint64_t raw = LoadLittleEndian(data);
  const uint64_t mask = (uint64_t{1} << bit_length) - 1U;
  return (raw >> start_bit) & mask;
}

int64_t ExtractSigned(const uint8_t* data, uint32_t start_bit,
                      uint32_t bit_length) {
  const uint64_t value = ExtractUnsigned(data, start_bit, bit_length);
  const uint64_t sign_bit = uint64_t{1} << (bit_length - 1U);
  if ((value & sign_bit) == 0U) {
    return static_cast<int64_t>(value);
  }
  const uint64_t extension_mask =
      std::numeric_limits<uint64_t>::max() << bit_length;
  return static_cast<int64_t>(value | extension_mask);
}

void ParseCcuStatus(const uint8_t* data, CcuStatus* status) {
  status->shift_status = ExtractUnsigned(data, 0, 2);
  status->parking_status = ExtractUnsigned(data, 2, 1) != 0U;
  status->ignition_status = ExtractUnsigned(data, 3, 2);
  status->drive_mode_shift_button = ExtractUnsigned(data, 5, 1) != 0U;
  status->steering_direction_right = ExtractUnsigned(data, 7, 1) != 0U;
  status->steering_magnitude = ExtractUnsigned(data, 8, 12) * 0.1;
  status->vehicle_speed_kph = ExtractUnsigned(data, 20, 9) * 0.1;
  status->drive_mode_raw = ExtractUnsigned(data, 29, 3);
  status->remote_brake = ExtractUnsigned(data, 32, 1) != 0U;
  status->emergency_brake = ExtractUnsigned(data, 33, 1) != 0U;
  status->scu_brake = ExtractUnsigned(data, 34, 1) != 0U;
  status->left_turn_light = ExtractUnsigned(data, 56, 1) != 0U;
  status->right_turn_light = ExtractUnsigned(data, 57, 1) != 0U;
  status->position_light = ExtractUnsigned(data, 59, 1) != 0U;
  status->low_beam = ExtractUnsigned(data, 60, 1) != 0U;
}

void ParseWheelSpeed(const uint8_t* data, WheelSpeedStatus* status) {
  status->front_left_rpm = ExtractSigned(data, 0, 16) * 0.1;
  status->front_right_rpm = ExtractSigned(data, 16, 16) * 0.1;
  status->rear_left_rpm = ExtractSigned(data, 32, 16) * 0.1;
  status->rear_right_rpm = ExtractSigned(data, 48, 16) * 0.1;
}

void ParseSteeringAngle(const uint8_t* data, SteeringAngleStatus* status) {
  status->front_angle_deg = ExtractSigned(data, 0, 16) * 0.1;
  status->rear_angle_deg = ExtractSigned(data, 24, 16) * 0.1;
}

void ParseTargetSpeed(const uint8_t* data, TargetSpeedStatus* status) {
  status->hardware_speed_kph = ExtractSigned(data, 0, 16) * 0.1;
  status->scu_speed_kph = ExtractSigned(data, 16, 16) * 0.1;
  status->vehicle_speed_kph = ExtractSigned(data, 32, 16) * 0.1;
  status->real_speed_rpm = ExtractSigned(data, 48, 16) * 0.1;
}

void ParseBmsStatus(const uint8_t* data, BmsStatus* status) {
  status->voltage_v = ExtractUnsigned(data, 0, 16) * 0.1;
  status->current_a = ExtractSigned(data, 16, 16) * 0.1;
  status->soc_percentage = ExtractUnsigned(data, 32, 16);
}

void ParseWarnings(const uint8_t* data, WarningStatus* status) {
  // JD03 V4.5 defines thirteen consecutive 3-bit severity fields. Although
  // its DBC screenshot marks them signed, the protocol values are levels
  // 0..3, so retain the raw unsigned values and never sign-extend them.
  status->bms_charge_current = ExtractUnsigned(data, 0, 3);
  status->bms_discharge_current = ExtractUnsigned(data, 3, 3);
  status->bms_soc = ExtractUnsigned(data, 6, 3);
  status->bms_temperature = ExtractUnsigned(data, 9, 3);
  status->mcu_current = ExtractUnsigned(data, 12, 3);
  status->mcu_disconnect = ExtractUnsigned(data, 15, 3);
  status->mcu_motor = ExtractUnsigned(data, 18, 3);
  status->mcu_speed = ExtractUnsigned(data, 21, 3);
  status->mcu_temperature = ExtractUnsigned(data, 24, 3);
  status->mcu_voltage = ExtractUnsigned(data, 27, 3);
  status->steering_disconnect = ExtractUnsigned(data, 30, 3);
  status->steering_lock = ExtractUnsigned(data, 33, 3);
  status->steering_unstoppable = ExtractUnsigned(data, 36, 3);
}

}  // namespace

uint32_t WarningStatus::MaxLevel() const {
  return std::max(
      {bms_charge_current, bms_discharge_current, bms_soc, bms_temperature,
       mcu_current, mcu_disconnect, mcu_motor, mcu_speed, mcu_temperature,
       mcu_voltage, steering_disconnect, steering_lock,
       steering_unstoppable});
}

uint32_t WarningStatus::ErrorMask() const {
  const std::array<uint32_t, 13> levels = {
      bms_charge_current, bms_discharge_current, bms_soc, bms_temperature,
      mcu_current, mcu_disconnect, mcu_motor, mcu_speed, mcu_temperature,
      mcu_voltage, steering_disconnect, steering_lock,
      steering_unstoppable};
  uint32_t mask = 0U;
  for (size_t i = 0; i < levels.size(); ++i) {
    if (levels[i] != 0U) {
      mask |= 1U << i;
    }
  }
  return mask;
}

bool IsRecognizedYunleCanId(uint32_t can_id) {
  switch (can_id) {
    case kCcuStatusId:
    case kWarningId:
    case kBmsStatusId:
    case kWheelSpeedId:
    case kSteeringAngleId:
    case kTargetSpeedId:
      return true;
    default:
      return false;
  }
}

bool ParseYunleFeedbackFrame(uint32_t can_id, const uint8_t* data,
                             size_t length, YunleChassisState* state) {
  if (data == nullptr || state == nullptr || length < 8U) {
    return false;
  }

  switch (can_id) {
    case kCcuStatusId:
      ParseCcuStatus(data, &state->ccu);
      return true;
    case kWarningId:
      ParseWarnings(data, &state->warning);
      return true;
    case kBmsStatusId:
      ParseBmsStatus(data, &state->bms);
      return true;
    case kWheelSpeedId:
      ParseWheelSpeed(data, &state->wheel_speed);
      return true;
    case kSteeringAngleId:
      ParseSteeringAngle(data, &state->steering);
      return true;
    case kTargetSpeedId:
      ParseTargetSpeed(data, &state->target_speed);
      return true;
    default:
      return false;
  }
}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
