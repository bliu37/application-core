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

#include <cstddef>
#include <cstdint>

namespace apollo {
namespace canbus {
namespace yunle {

constexpr uint32_t kCcuStatusId = 0x51U;
constexpr uint32_t kWarningId = 0x77U;
constexpr uint32_t kBmsStatusId = 0x100U;
constexpr uint32_t kWheelSpeedId = 0x168U;
constexpr uint32_t kSteeringAngleId = 0xE1U;
constexpr uint32_t kTargetSpeedId = 0x7F1U;

struct CcuStatus {
  uint32_t shift_status = 0;
  bool parking_status = false;
  uint32_t ignition_status = 0;
  bool drive_mode_shift_button = false;
  bool steering_direction_right = false;
  double steering_magnitude = 0.0;
  double vehicle_speed_kph = 0.0;
  uint32_t drive_mode_raw = 0;
  bool remote_brake = false;
  bool emergency_brake = false;
  bool scu_brake = false;
  bool left_turn_light = false;
  bool right_turn_light = false;
  bool position_light = false;
  bool low_beam = false;
};

struct WheelSpeedStatus {
  double front_left_rpm = 0.0;
  double front_right_rpm = 0.0;
  double rear_left_rpm = 0.0;
  double rear_right_rpm = 0.0;
};

struct SteeringAngleStatus {
  double front_angle_deg = 0.0;
  double rear_angle_deg = 0.0;
};

struct TargetSpeedStatus {
  double hardware_speed_kph = 0.0;
  double scu_speed_kph = 0.0;
  double vehicle_speed_kph = 0.0;
  double real_speed_rpm = 0.0;
};

struct BmsStatus {
  double voltage_v = 0.0;
  double current_a = 0.0;
  uint32_t soc_percentage = 0;
};

struct WarningStatus {
  uint32_t bms_charge_current = 0;
  uint32_t bms_discharge_current = 0;
  uint32_t bms_soc = 0;
  uint32_t bms_temperature = 0;
  uint32_t mcu_current = 0;
  uint32_t mcu_disconnect = 0;
  uint32_t mcu_motor = 0;
  uint32_t mcu_speed = 0;
  uint32_t mcu_temperature = 0;
  uint32_t mcu_voltage = 0;
  uint32_t steering_disconnect = 0;
  uint32_t steering_lock = 0;
  uint32_t steering_unstoppable = 0;

  uint32_t MaxLevel() const;
  uint32_t ErrorMask() const;
};

struct YunleChassisState {
  CcuStatus ccu;
  WheelSpeedStatus wheel_speed;
  SteeringAngleStatus steering;
  TargetSpeedStatus target_speed;
  BmsStatus bms;
  WarningStatus warning;
};

bool IsRecognizedYunleCanId(uint32_t can_id);

// Parses only receive-side feedback IDs. Control IDs 0x121 and 0x123 are
// intentionally absent from this interface.
bool ParseYunleFeedbackFrame(uint32_t can_id, const uint8_t* data,
                             size_t length, YunleChassisState* state);

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
