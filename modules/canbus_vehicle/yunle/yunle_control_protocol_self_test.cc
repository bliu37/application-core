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

#include <array>
#include <iostream>
#include <limits>
#include <string>

namespace apollo {
namespace canbus {
namespace yunle {

namespace {

using ControlData = std::array<uint8_t, kScuControlLength>;

bool CheckEncoding(const std::string& name,
                   const YunleScuControlCommand& command,
                   const ControlData& expected) {
  ControlData actual {};
  if (!EncodeScuControl121(command, &actual)) {
    std::cerr << "[FAILED] " << name << ": encoder rejected command\n";
    return false;
  }
  if (actual != expected) {
    std::cerr << "[FAILED] " << name << ": encoded bytes differ\n";
    return false;
  }
  std::cout << "[PASSED] " << name << "\n";
  return true;
}

bool CheckInvalidInputs() {
  YunleScuControlCommand command;
  ControlData data {};

  command.target_speed_kph = -0.1;
  if (EncodeScuControl121(command, &data)) {
    return false;
  }
  command.target_speed_kph = 51.2;
  if (EncodeScuControl121(command, &data)) {
    return false;
  }
  command.target_speed_kph = std::numeric_limits<double>::quiet_NaN();
  if (EncodeScuControl121(command, &data)) {
    return false;
  }
  command.target_speed_kph = 0.0;
  command.front_steering_percentage = 100.1;
  if (EncodeScuControl121(command, &data)) {
    return false;
  }
  command.front_steering_percentage = 0.0;
  command.drive_mode = static_cast<YunleDriveModeRequest>(2U);
  return !EncodeScuControl121(command, &data) &&
         !EncodeScuControl121(command, nullptr);
}

}  // namespace

int RunSelfTest() {
  int passed = 0;

  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.brake_enable = true;
  passed += CheckEncoding(
      "V4.5 brake", command,
      {0x40, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00});

  command = YunleScuControlCommand();
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.front_steering_percentage = 50.0;
  command.rear_steering_percentage = 50.0;
  passed += CheckEncoding(
      "V4.5 left steering", command,
      {0x40, 0xC4, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00});

  command.front_steering_percentage = -50.0;
  command.rear_steering_percentage = -50.0;
  passed += CheckEncoding(
      "V4.5 right steering", command,
      {0x40, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00});

  command = YunleScuControlCommand();
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.gear = YunleGearRequest::kDrive;
  command.target_speed_kph = 3.0;
  passed += CheckEncoding(
      "V4.5 3 km/h drive", command,
      {0x41, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00});

  command.gear = YunleGearRequest::kReverse;
  passed += CheckEncoding(
      "V4.5 3 km/h reverse", command,
      {0x43, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00});

  command.gear = YunleGearRequest::kDrive;
  command.target_speed_kph = 0.3;
  passed += CheckEncoding(
      "0.3 km/h request encoded exactly", command,
      {0x41, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00});

  command.target_speed_kph = 0.36;
  passed += CheckEncoding(
      "0.36 km/h request quantized down", command,
      {0x41, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00});

  if (CheckInvalidInputs()) {
    std::cout << "[PASSED] invalid input rejection\n";
    ++passed;
  } else {
    std::cerr << "[FAILED] invalid input rejection\n";
  }

  if (passed != 8) {
    std::cerr << "[FAILED] " << passed << " of 8 checks passed\n";
    return 1;
  }
  std::cout << "[PASSED] all 8 checks\n";
  return 0;
}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo

int main() { return apollo::canbus::yunle::RunSelfTest(); }
