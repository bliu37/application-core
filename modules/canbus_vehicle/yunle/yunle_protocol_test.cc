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

#include <array>

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace yunle {

TEST(YunleProtocolTest, ParsesCapturedCcuStatus) {
  const std::array<uint8_t, 8> data = {
      0x2A, 0x0D, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(kCcuStatusId, data.data(), data.size(),
                                      &state));
  EXPECT_EQ(2U, state.ccu.shift_status);
  EXPECT_FALSE(state.ccu.parking_status);
  EXPECT_EQ(1U, state.ccu.ignition_status);
  EXPECT_TRUE(state.ccu.drive_mode_shift_button);
  EXPECT_FALSE(state.ccu.steering_direction_right);
  EXPECT_DOUBLE_EQ(1.3, state.ccu.steering_magnitude);
  EXPECT_DOUBLE_EQ(0.0, state.ccu.vehicle_speed_kph);
  EXPECT_EQ(3U, state.ccu.drive_mode_raw);
}

TEST(YunleProtocolTest, ParsesCapturedBmsStatus) {
  const std::array<uint8_t, 8> data = {
      0x11, 0x02, 0xED, 0xFF, 0x53, 0x00, 0x00, 0x00};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(kBmsStatusId, data.data(), data.size(),
                                      &state));
  EXPECT_DOUBLE_EQ(52.9, state.bms.voltage_v);
  EXPECT_DOUBLE_EQ(-1.9, state.bms.current_a);
  EXPECT_EQ(83U, state.bms.soc_percentage);
}

TEST(YunleProtocolTest, ParsesCapturedSteeringAngles) {
  const std::array<uint8_t, 8> data = {
      0x0D, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(
      kSteeringAngleId, data.data(), data.size(), &state));
  EXPECT_DOUBLE_EQ(1.3, state.steering.front_angle_deg);
  EXPECT_DOUBLE_EQ(1.1, state.steering.rear_angle_deg);
}

TEST(YunleProtocolTest, ParsesSignedWheelSpeeds) {
  const std::array<uint8_t, 8> data = {
      0x64, 0x00, 0x9C, 0xFF, 0xC8, 0x00, 0x38, 0xFF};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(
      kWheelSpeedId, data.data(), data.size(), &state));
  EXPECT_DOUBLE_EQ(10.0, state.wheel_speed.front_left_rpm);
  EXPECT_DOUBLE_EQ(-10.0, state.wheel_speed.front_right_rpm);
  EXPECT_DOUBLE_EQ(20.0, state.wheel_speed.rear_left_rpm);
  EXPECT_DOUBLE_EQ(-20.0, state.wheel_speed.rear_right_rpm);
}

TEST(YunleProtocolTest, ParsesTargetSpeedsAndRealSpeed) {
  const std::array<uint8_t, 8> data = {
      0x64, 0x00, 0x9C, 0xFF, 0xC8, 0x00, 0x38, 0xFF};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(kTargetSpeedId, data.data(), data.size(),
                                      &state));
  EXPECT_DOUBLE_EQ(10.0, state.target_speed.hardware_speed_kph);
  EXPECT_DOUBLE_EQ(-10.0, state.target_speed.scu_speed_kph);
  EXPECT_DOUBLE_EQ(20.0, state.target_speed.vehicle_speed_kph);
  EXPECT_DOUBLE_EQ(-20.0, state.target_speed.real_speed_rpm);
}

TEST(YunleProtocolTest, ParsesAllV45WarningLevelsAsUnsigned) {
  const std::array<uint8_t, 8> data = {
      0xD1, 0x10, 0x0D, 0xD1, 0x22, 0x00, 0x00, 0x00};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(kWarningId, data.data(), data.size(),
                                      &state));
  EXPECT_EQ(1U, state.warning.bms_charge_current);
  EXPECT_EQ(2U, state.warning.bms_discharge_current);
  EXPECT_EQ(3U, state.warning.bms_soc);
  EXPECT_EQ(0U, state.warning.bms_temperature);
  EXPECT_EQ(1U, state.warning.mcu_current);
  EXPECT_EQ(2U, state.warning.mcu_disconnect);
  EXPECT_EQ(3U, state.warning.mcu_motor);
  EXPECT_EQ(0U, state.warning.mcu_speed);
  EXPECT_EQ(1U, state.warning.mcu_temperature);
  EXPECT_EQ(2U, state.warning.mcu_voltage);
  EXPECT_EQ(3U, state.warning.steering_disconnect);
  EXPECT_EQ(1U, state.warning.steering_lock);
  EXPECT_EQ(2U, state.warning.steering_unstoppable);
  EXPECT_EQ(3U, state.warning.MaxLevel());
  EXPECT_EQ(0x1F77U, state.warning.ErrorMask());
}

TEST(YunleProtocolTest, KeepsOutOfRangeWarningLevelsUnsigned) {
  const std::array<uint8_t, 8> data = {
      0x07, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00};
  YunleChassisState state;

  ASSERT_TRUE(ParseYunleFeedbackFrame(kWarningId, data.data(), data.size(),
                                      &state));
  EXPECT_EQ(7U, state.warning.bms_charge_current);
  EXPECT_EQ(7U, state.warning.steering_unstoppable);
  EXPECT_EQ(7U, state.warning.MaxLevel());
  EXPECT_EQ((1U << 0U) | (1U << 12U), state.warning.ErrorMask());
}

TEST(YunleProtocolTest, RejectsShortAndControlFrames) {
  const std::array<uint8_t, 8> data = {};
  YunleChassisState state;

  EXPECT_FALSE(ParseYunleFeedbackFrame(kCcuStatusId, data.data(), 7U, &state));
  EXPECT_FALSE(ParseYunleFeedbackFrame(0x121U, data.data(), data.size(),
                                       &state));
  EXPECT_FALSE(IsRecognizedYunleCanId(0x121U));
  EXPECT_FALSE(IsRecognizedYunleCanId(0x123U));
}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
