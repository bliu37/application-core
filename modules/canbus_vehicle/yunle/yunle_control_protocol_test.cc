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
#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace yunle {

TEST(YunleControlProtocolTest, EncodesV45BrakeExample) {
  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.brake_enable = true;
  std::array<uint8_t, kScuControlLength> data {};

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  const std::array<uint8_t, kScuControlLength> expected = {
      0x40, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  EXPECT_EQ(expected, data);
}

TEST(YunleControlProtocolTest, EncodesV45LeftSteeringExample) {
  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.front_steering_percentage = 50.0;
  command.rear_steering_percentage = 50.0;
  std::array<uint8_t, kScuControlLength> data {};

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  const std::array<uint8_t, kScuControlLength> expected = {
      0x40, 0xC4, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(expected, data);
}

TEST(YunleControlProtocolTest, EncodesV45RightSteeringExample) {
  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.front_steering_percentage = -50.0;
  command.rear_steering_percentage = -50.0;
  std::array<uint8_t, kScuControlLength> data {};

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  const std::array<uint8_t, kScuControlLength> expected = {
      0x40, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(expected, data);
}

TEST(YunleControlProtocolTest, EncodesV45DriveAndReverseExamples) {
  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.gear = YunleGearRequest::kDrive;
  command.target_speed_kph = 3.0;
  std::array<uint8_t, kScuControlLength> data {};

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  const std::array<uint8_t, kScuControlLength> drive_expected = {
      0x41, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(drive_expected, data);

  command.gear = YunleGearRequest::kReverse;
  ASSERT_TRUE(EncodeScuControl121(command, &data));
  const std::array<uint8_t, kScuControlLength> reverse_expected = {
      0x43, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(reverse_expected, data);
}

TEST(YunleControlProtocolTest, QuantizesSpeedDownWithoutOvershoot) {
  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.gear = YunleGearRequest::kDrive;
  command.target_speed_kph = 0.3;
  std::array<uint8_t, kScuControlLength> data {};

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  EXPECT_EQ(0x03U, data[3]);
  EXPECT_EQ(0x00U, data[4] & 0x01U);

  command.target_speed_kph = 0.36;

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  EXPECT_EQ(0x03U, data[3]);
  EXPECT_EQ(0x00U, data[4] & 0x01U);
}

TEST(YunleControlProtocolTest, EncodesCrossByteSpeedAndFullSteering) {
  YunleScuControlCommand command;
  command.drive_mode = YunleDriveModeRequest::kAutonomous;
  command.gear = YunleGearRequest::kNeutral;
  command.target_speed_kph = 51.1;
  command.front_steering_percentage = 100.0;
  command.rear_steering_percentage = -100.0;
  command.brake_enable = true;
  std::array<uint8_t, kScuControlLength> data {};

  ASSERT_TRUE(EncodeScuControl121(command, &data));
  const std::array<uint8_t, kScuControlLength> expected = {
      0x42, 0x88, 0x78, 0xFF, 0x03, 0x00, 0x00, 0x00};
  EXPECT_EQ(expected, data);
}

TEST(YunleControlProtocolTest, RejectsInvalidInputs) {
  YunleScuControlCommand command;
  std::array<uint8_t, kScuControlLength> data {};

  command.target_speed_kph = -0.1;
  EXPECT_FALSE(EncodeScuControl121(command, &data));
  command.target_speed_kph = 51.2;
  EXPECT_FALSE(EncodeScuControl121(command, &data));
  command.target_speed_kph = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(EncodeScuControl121(command, &data));

  command.target_speed_kph = 0.0;
  command.front_steering_percentage = 100.1;
  EXPECT_FALSE(EncodeScuControl121(command, &data));
  command.front_steering_percentage = 0.0;
  command.left_turn_light_request = 4U;
  EXPECT_FALSE(EncodeScuControl121(command, &data));

  command.left_turn_light_request = 0U;
  command.drive_mode = static_cast<YunleDriveModeRequest>(2U);
  EXPECT_FALSE(EncodeScuControl121(command, &data));
  EXPECT_FALSE(EncodeScuControl121(command, nullptr));
}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
