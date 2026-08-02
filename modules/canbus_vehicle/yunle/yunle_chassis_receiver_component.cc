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

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/canbus_vehicle/yunle/proto/yunle_chassis_receiver.pb.h"
#include "modules/canbus_vehicle/yunle/yunle_control_protocol.h"
#include "modules/canbus_vehicle/yunle/yunle_protocol.h"
#include "modules/common/util/message_util.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

namespace apollo {
namespace canbus {
namespace yunle {

namespace {

constexpr size_t kSerializedFrameSize = 13U;
constexpr uint8_t kDlcMask = 0x0FU;
constexpr uint8_t kRemoteFrameFlag = 0x40U;
constexpr uint8_t kExtendedFrameFlag = 0x80U;
constexpr uint32_t kStandardCanIdMask = 0x7FFU;
constexpr uint32_t kExtendedCanIdMask = 0x1FFFFFFFU;
constexpr double kPi = 3.14159265358979323846;
constexpr int kSocketReceiveBufferSize = 4 * 1024 * 1024;
constexpr double kAbsoluteControlCommandSpeedLimitKph = 1.5;
constexpr double kAbsoluteMeasuredSpeedLimitKph = 1.7;
constexpr double kAbsoluteControlSteeringLimitPercentage = 90.0;
constexpr double kControlCommandSpeedNegativeToleranceMps = 0.02;
constexpr double kControlCommandSpeedUpperToleranceKph = 0.05;
constexpr double kStationaryFailsafeSuppressSpeedKph = 0.05;
constexpr uint32_t kAbsoluteControlCommandTimeoutMs = 200U;
constexpr uint32_t kAbsoluteFailsafeSendMs = 500U;

double Clamp(double value, double lower, double upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

Chassis::GearPosition ToApolloGear(uint32_t shift_status,
                                   bool parking_status) {
  if (parking_status) {
    return Chassis::GEAR_PARKING;
  }
  switch (shift_status) {
    case 1U:
      return Chassis::GEAR_DRIVE;
    case 2U:
      return Chassis::GEAR_NEUTRAL;
    case 3U:
      return Chassis::GEAR_REVERSE;
    default:
      return Chassis::GEAR_INVALID;
  }
}

WheelSpeed::WheelSpeedType ToWheelDirection(double rpm) {
  constexpr double kStationaryRpmThreshold = 0.1;
  if (rpm > kStationaryRpmThreshold) {
    return WheelSpeed::FORWARD;
  }
  if (rpm < -kStationaryRpmThreshold) {
    return WheelSpeed::BACKWARD;
  }
  return WheelSpeed::STANDSTILL;
}

}  // namespace

class YunleChassisReceiverComponent final
    : public apollo::cyber::TimerComponent {
 public:
  ~YunleChassisReceiverComponent() override { CloseSocket(); }

  bool Init() override {
    if (!GetProtoConfig(&config_)) {
      AERROR << "Unable to load Yunle chassis receiver config: "
             << ConfigFilePath();
      return false;
    }
    if (!ValidateConfig()) {
      return false;
    }

    chassis_writer_ = node_->CreateWriter<Chassis>(config_.chassis_topic());
    detail_writer_ = node_->CreateWriter<YunleChassisDetail>(
        config_.detail_topic());
    if (chassis_writer_ == nullptr || detail_writer_ == nullptr) {
      AERROR << "Failed to create Yunle chassis output writers.";
      return false;
    }
    if (config_.enable_control_send()) {
      control_reader_ = node_->CreateReader<control::ControlCommand>(
          config_.control_topic(),
          [this](const std::shared_ptr<control::ControlCommand>& command) {
            OnControlCommand(command);
          });
      if (control_reader_ == nullptr) {
        AERROR << "Failed to create Yunle control command reader.";
        return false;
      }
    }
    if (!OpenReceiveSocket()) {
      return false;
    }

    if (config_.enable_control_send()) {
      AWARN << "Yunle 0x121 control transmission is ENABLED. Hard limits: "
            << config_.maximum_control_speed_kph() << " km/h and "
            << config_.maximum_control_steering_percentage()
            << "% steering. UDP peer: " << config_.remote_ip() << ":"
            << config_.remote_port();
    } else {
      AWARN << "Yunle control transmitter is compiled but DISABLED by "
            << "Profile. Component remains receive-only. Listening on "
            << config_.local_ip() << ":" << config_.local_port()
            << " for peer " << config_.remote_ip() << ":"
            << config_.remote_port();
    }
    return true;
  }

  bool Proc() override {
    ReceivePendingDatagrams();
    const auto now = Clock::now();
    MaybeSendControl(now);
    PublishChassis(now);
    PublishDetail(now);
    return true;
  }

  void Clear() override { CloseSocket(); }

 private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  bool ValidateConfig() {
    if (config_.local_port() > 65535U || config_.remote_port() > 65535U) {
      AERROR << "Yunle UDP port is outside uint16 range.";
      return false;
    }
    if (config_.local_ip().empty() || config_.remote_ip().empty() ||
        config_.chassis_topic().empty() || config_.detail_topic().empty() ||
        config_.control_topic().empty()) {
      AERROR << "Yunle chassis receiver config contains an empty address or "
                "topic.";
      return false;
    }
    if (config_.max_steering_magnitude() <= 0.0 ||
        config_.wheel_radius_m() <= 0.0 ||
        config_.feedback_timeout_ms() == 0U ||
        config_.bms_timeout_ms() == 0U ||
        config_.max_datagrams_per_cycle() == 0U ||
        config_.control_command_timeout_ms() == 0U ||
        config_.control_command_timeout_ms() >
            kAbsoluteControlCommandTimeoutMs ||
        config_.failsafe_send_ms() == 0U ||
        config_.failsafe_send_ms() > kAbsoluteFailsafeSendMs) {
      AERROR << "Yunle chassis receiver config contains an invalid physical "
                "or timeout parameter.";
      return false;
    }
    if (!std::isfinite(config_.maximum_control_speed_kph()) ||
        config_.maximum_control_speed_kph() <= 0.0 ||
        config_.maximum_control_speed_kph() >
            kAbsoluteControlCommandSpeedLimitKph ||
        !std::isfinite(config_.maximum_control_steering_percentage()) ||
        config_.maximum_control_steering_percentage() <= 0.0 ||
        config_.maximum_control_steering_percentage() >
            kAbsoluteControlSteeringLimitPercentage) {
      AERROR << "Yunle control limit exceeds the compiled low-speed safety "
                "ceiling.";
      return false;
    }
    if (config_.control_info_byte() > 0xFFU ||
        (config_.control_info_byte() &
         (kRemoteFrameFlag | kExtendedFrameFlag)) != 0U) {
      AERROR << "Invalid Yunle ETH-CAN control Info byte.";
      return false;
    }

    if (inet_pton(AF_INET, config_.remote_ip().c_str(),
                  &expected_remote_address_) != 1) {
      AERROR << "Invalid Yunle remote IPv4 address: " << config_.remote_ip();
      return false;
    }
    expected_remote_port_ = htons(static_cast<uint16_t>(config_.remote_port()));
    remote_address_.sin_family = AF_INET;
    remote_address_.sin_addr = expected_remote_address_;
    remote_address_.sin_port = expected_remote_port_;
    return true;
  }

  bool OpenReceiveSocket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      AERROR << "Failed to create Yunle chassis UDP socket: "
             << std::strerror(errno);
      return false;
    }

    const int current_flags = fcntl(socket_fd_, F_GETFL, 0);
    if (current_flags < 0 ||
        fcntl(socket_fd_, F_SETFL, current_flags | O_NONBLOCK) < 0) {
      AERROR << "Failed to make Yunle UDP socket non-blocking: "
             << std::strerror(errno);
      CloseSocket();
      return false;
    }

    int receive_buffer_size = kSocketReceiveBufferSize;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &receive_buffer_size,
                   sizeof(receive_buffer_size)) < 0) {
      AWARN << "Unable to enlarge Yunle UDP receive buffer: "
            << std::strerror(errno);
    }

    sockaddr_in local_address {};
    local_address.sin_family = AF_INET;
    local_address.sin_port =
        htons(static_cast<uint16_t>(config_.local_port()));
    if (inet_pton(AF_INET, config_.local_ip().c_str(),
                  &local_address.sin_addr) != 1) {
      AERROR << "Invalid Yunle local IPv4 address: " << config_.local_ip();
      CloseSocket();
      return false;
    }
    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&local_address),
             sizeof(local_address)) < 0) {
      AERROR << "Failed to bind Yunle chassis UDP socket at "
             << config_.local_ip() << ":" << config_.local_port() << ": "
             << std::strerror(errno)
             << ". Stop the old ROS can_bridge or another chassis receiver.";
      CloseSocket();
      return false;
    }
    return true;
  }

  void CloseSocket() {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  bool IsExpectedPeer(const sockaddr_in& peer) const {
    return peer.sin_addr.s_addr == expected_remote_address_.s_addr &&
           peer.sin_port == expected_remote_port_;
  }

  void ReceivePendingDatagrams() {
    if (socket_fd_ < 0) {
      return;
    }

    for (uint32_t i = 0; i < config_.max_datagrams_per_cycle(); ++i) {
      sockaddr_in peer {};
      socklen_t peer_length = sizeof(peer);
      const ssize_t received = recvfrom(
          socket_fd_, receive_buffer_.data(), receive_buffer_.size(), 0,
          reinterpret_cast<sockaddr*>(&peer), &peer_length);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        AERROR_EVERY(100) << "Yunle UDP receive failed: "
                          << std::strerror(errno);
        break;
      }
      if (!IsExpectedPeer(peer)) {
        ++unexpected_peer_count_;
        continue;
      }
      if (received == 0 ||
          static_cast<size_t>(received) % kSerializedFrameSize != 0U) {
        ++invalid_packet_count_;
        continue;
      }

      const size_t packet_length = static_cast<size_t>(received);
      for (size_t offset = 0; offset < packet_length;
           offset += kSerializedFrameSize) {
        ParseSerializedFrame(receive_buffer_.data() + offset);
      }
    }
  }

  void ParseSerializedFrame(const uint8_t* frame) {
    const uint8_t info = frame[0];
    const size_t dlc = info & kDlcMask;
    if (dlc > 8U || (info & kRemoteFrameFlag) != 0U) {
      ++invalid_packet_count_;
      return;
    }

    uint32_t can_id = (static_cast<uint32_t>(frame[1]) << 24U) |
                      (static_cast<uint32_t>(frame[2]) << 16U) |
                      (static_cast<uint32_t>(frame[3]) << 8U) |
                      static_cast<uint32_t>(frame[4]);
    can_id &= (info & kExtendedFrameFlag) != 0U ? kExtendedCanIdMask
                                                : kStandardCanIdMask;

    ++received_frame_count_;
    last_info_byte_ = info;
    if (!IsRecognizedYunleCanId(can_id)) {
      return;
    }
    if (!ParseYunleFeedbackFrame(can_id, frame + 5U, dlc, &state_)) {
      ++invalid_packet_count_;
      return;
    }

    ++recognized_frame_count_;
    const TimePoint now = Clock::now();
    switch (can_id) {
      case kCcuStatusId:
        has_ccu_ = true;
        last_ccu_ = now;
        break;
      case kWarningId:
        has_warning_ = true;
        last_warning_ = now;
        break;
      case kBmsStatusId:
        has_bms_ = true;
        last_bms_ = now;
        break;
      case kWheelSpeedId:
        has_wheel_speed_ = true;
        last_wheel_speed_ = now;
        break;
      case kSteeringAngleId:
        has_steering_ = true;
        last_steering_ = now;
        break;
      case kTargetSpeedId:
        has_target_speed_ = true;
        last_target_speed_ = now;
        break;
      default:
        break;
    }
  }

  uint32_t AgeMs(bool has_value, const TimePoint& timestamp,
                 const TimePoint& now) const {
    if (!has_value) {
      return std::numeric_limits<uint32_t>::max();
    }
    const auto age =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp)
            .count();
    if (age <= 0) {
      return 0U;
    }
    if (static_cast<uint64_t>(age) >=
        std::numeric_limits<uint32_t>::max()) {
      return std::numeric_limits<uint32_t>::max();
    }
    return static_cast<uint32_t>(age);
  }

  bool IsFresh(bool has_value, const TimePoint& timestamp, uint32_t timeout_ms,
               const TimePoint& now) const {
    return has_value && AgeMs(true, timestamp, now) <= timeout_ms;
  }

  bool CoreFeedbackIsFresh(const TimePoint& now) const {
    const uint32_t timeout_ms = config_.feedback_timeout_ms();
    return IsFresh(has_ccu_, last_ccu_, timeout_ms, now) &&
           IsFresh(has_wheel_speed_, last_wheel_speed_, timeout_ms, now) &&
           IsFresh(has_steering_, last_steering_, timeout_ms, now) &&
           IsFresh(has_warning_, last_warning_, timeout_ms, now);
  }

  void OnControlCommand(
      const std::shared_ptr<control::ControlCommand>& command) {
    if (command == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(control_command_mutex_);
    latest_control_command_.CopyFrom(*command);
    has_control_command_ = true;
    last_control_command_ = Clock::now();
  }

  bool GetFreshControlCommand(const TimePoint& now,
                              control::ControlCommand* command) {
    std::lock_guard<std::mutex> lock(control_command_mutex_);
    control_command_received_ = has_control_command_;
    if (!has_control_command_ ||
        !IsFresh(true, last_control_command_,
                 config_.control_command_timeout_ms(), now)) {
      return false;
    }
    command->CopyFrom(latest_control_command_);
    return true;
  }

  bool ControlInterlocksAreSatisfied(const TimePoint& now,
                                     std::string* reason) const {
    if (!CoreFeedbackIsFresh(now)) {
      *reason = "core chassis feedback is stale";
      return false;
    }
    if (state_.warning.MaxLevel() >= 2U) {
      *reason = "chassis warning level is 2 or higher";
      return false;
    }
    if (!std::isfinite(state_.ccu.vehicle_speed_kph) ||
        std::abs(state_.ccu.vehicle_speed_kph) >
            kAbsoluteMeasuredSpeedLimitKph) {
      *reason = "measured chassis speed exceeds the low-speed safety limit";
      return false;
    }
    // The chassis also raises parking_status when an SCU brake command is
    // active. Validate the independent raw shift field here so that a valid
    // zero-speed brake command does not trip its own gear interlock.
    if (state_.ccu.shift_status < 1U || state_.ccu.shift_status > 3U) {
      *reason = "chassis feedback gear is not D, N, or R";
      return false;
    }
    if (config_.require_drive_mode_switch() &&
        !state_.ccu.drive_mode_shift_button) {
      *reason = "physical autonomous-drive switch is not pressed";
      return false;
    }
    return true;
  }

  bool ControlCommandAllowsScuControl(const control::ControlCommand& input,
                                      std::string* reason) const {
    if (input.has_header() && input.header().has_status() &&
        input.header().status().error_code() != common::OK) {
      *reason = "ControlCommand status is not OK";
      return false;
    }
    if (input.has_engage_advice() &&
        input.engage_advice().advice() !=
            common::EngageAdvice::READY_TO_ENGAGE &&
        input.engage_advice().advice() !=
            common::EngageAdvice::KEEP_ENGAGED) {
      *reason = "ControlCommand engage advice does not allow control";
      return false;
    }
    return true;
  }

  bool HardwareOverrideBlocksControl(std::string* reason) const {
    if (state_.ccu.remote_brake || state_.ccu.emergency_brake) {
      *reason = "hardware brake input is active";
      return true;
    }
    if (config_.require_drive_mode_switch() &&
        !state_.ccu.drive_mode_shift_button) {
      *reason = "physical autonomous-drive switch is not pressed";
      return true;
    }
    return false;
  }

  bool BuildScuControlCommand(const control::ControlCommand& input,
                              YunleScuControlCommand* output,
                              std::string* reason) const {
    if (!input.has_gear_location() || !input.has_speed() ||
        !input.has_steering_target()) {
      *reason = "ControlCommand lacks gear, speed, or steering target";
      return false;
    }
    if (!std::isfinite(input.speed()) ||
        !std::isfinite(input.steering_target()) ||
        input.speed() < -kControlCommandSpeedNegativeToleranceMps ||
        (input.has_brake() &&
         (!std::isfinite(input.brake()) || input.brake() < 0.0 ||
          input.brake() > 100.0))) {
      *reason = "ControlCommand contains an invalid numeric value";
      return false;
    }
    if (input.is_in_safe_mode()) {
      *reason = "ControlCommand reports safe mode";
      return false;
    }

    switch (input.gear_location()) {
      case Chassis::GEAR_DRIVE:
        output->gear = YunleGearRequest::kDrive;
        break;
      case Chassis::GEAR_NEUTRAL:
        output->gear = YunleGearRequest::kNeutral;
        break;
      case Chassis::GEAR_REVERSE:
        output->gear = YunleGearRequest::kReverse;
        break;
      default:
        *reason = "ControlCommand gear is not D, N, or R";
        return false;
    }

    const double requested_speed_kph =
        (input.speed() < 0.0 ? 0.0 : input.speed()) * 3.6;
    if (requested_speed_kph > config_.maximum_control_speed_kph() +
                                  kControlCommandSpeedUpperToleranceKph) {
      *reason = "ControlCommand exceeds the configured speed limit";
      return false;
    }
    const double target_speed_kph =
        Clamp(requested_speed_kph, 0.0,
              config_.maximum_control_speed_kph());
    if (target_speed_kph > 0.0 &&
        output->gear == YunleGearRequest::kNeutral) {
      *reason = "nonzero speed is not allowed in neutral";
      return false;
    }
    if (std::abs(input.steering_target()) >
        config_.maximum_control_steering_percentage()) {
      *reason = "ControlCommand exceeds the configured steering limit";
      return false;
    }

    output->drive_mode = YunleDriveModeRequest::kAutonomous;
    output->target_speed_kph = target_speed_kph;
    output->front_steering_percentage = input.steering_target();
    output->rear_steering_percentage = input.steering_target();
    output->brake_enable =
        (input.has_parking_brake() && input.parking_brake()) ||
        (input.has_brake() && input.brake() > 0.0);
    // Apollo Control may request a positive target speed immediately after it
    // decides to release EPB.  The Yunle SCU needs one or more zero-speed
    // frames with brake released before parking_status drops.  Keep the command
    // valid and release-only until the chassis reports parking released; then
    // the next fresh ControlCommand can carry the nonzero target speed.
    if (output->brake_enable ||
        (target_speed_kph > 0.0 && state_.ccu.parking_status)) {
      output->target_speed_kph = 0.0;
    }

    if (input.has_signal()) {
      if (input.signal().turn_signal() ==
          common::VehicleSignal::TURN_LEFT) {
        output->left_turn_light_request = 1U;
      } else if (input.signal().turn_signal() ==
                 common::VehicleSignal::TURN_RIGHT) {
        output->right_turn_light_request = 1U;
      } else if (input.signal().turn_signal() ==
                 common::VehicleSignal::TURN_HAZARD_WARNING) {
        output->left_turn_light_request = 1U;
        output->right_turn_light_request = 1U;
      }
      output->low_beam_request = input.signal().low_beam() ? 1U : 0U;
    }
    return true;
  }

  bool SendScuControl121(const YunleScuControlCommand& command) {
    std::array<uint8_t, kScuControlLength> data {};
    if (!EncodeScuControl121(command, &data)) {
      ++control_send_error_count_;
      AERROR_EVERY(100) << "Failed to encode Yunle 0x121 control command.";
      return false;
    }

    std::array<uint8_t, kSerializedFrameSize> frame {};
    frame[0] = static_cast<uint8_t>(
        (config_.control_info_byte() & 0xF0U) | kScuControlLength);
    frame[1] = static_cast<uint8_t>((kScuControlId >> 24U) & 0xFFU);
    frame[2] = static_cast<uint8_t>((kScuControlId >> 16U) & 0xFFU);
    frame[3] = static_cast<uint8_t>((kScuControlId >> 8U) & 0xFFU);
    frame[4] = static_cast<uint8_t>(kScuControlId & 0xFFU);
    std::memcpy(frame.data() + 5U, data.data(), data.size());

    const ssize_t sent = sendto(
        socket_fd_, frame.data(), frame.size(), 0,
        reinterpret_cast<const sockaddr*>(&remote_address_),
        sizeof(remote_address_));
    if (sent != static_cast<ssize_t>(frame.size())) {
      ++control_send_error_count_;
      if (sent < 0) {
        AERROR_EVERY(100) << "Failed to send Yunle 0x121 UDP frame: "
                          << std::strerror(errno);
      } else {
        AERROR_EVERY(100) << "Short Yunle 0x121 UDP send: " << sent
                          << " of " << frame.size() << " bytes.";
      }
      return false;
    }
    ++sent_control_frame_count_;
    commanded_target_speed_kph_ = command.target_speed_kph;
    commanded_steering_percentage_ =
        command.front_steering_percentage;
    commanded_gear_ = static_cast<uint32_t>(command.gear);
    return true;
  }

  void StartFailsafe(const TimePoint& now) {
    if (!control_failsafe_active_) {
      control_failsafe_active_ = true;
      failsafe_started_ = now;
    }
  }

  void MaybeSendControl(const TimePoint& now) {
    control_command_fresh_ = false;
    control_interlocks_ok_ = false;
    if (!config_.enable_control_send()) {
      control_block_reason_ = "control send disabled by Profile";
      return;
    }

    control::ControlCommand input;
    control_command_fresh_ = GetFreshControlCommand(now, &input);
    if (CoreFeedbackIsFresh(now) &&
        HardwareOverrideBlocksControl(&control_block_reason_)) {
      control_active_ = false;
      control_failsafe_active_ = false;
      return;
    }
    if (!control_command_fresh_) {
      control_block_reason_ = "no fresh ControlCommand";
    } else {
      control_interlocks_ok_ =
          ControlInterlocksAreSatisfied(now, &control_block_reason_);
    }

    bool control_command_allows_scu = false;
    if (control_command_fresh_ && control_interlocks_ok_) {
      control_command_allows_scu =
          ControlCommandAllowsScuControl(input, &control_block_reason_);
    }

    YunleScuControlCommand command;
    const bool command_is_valid =
        control_command_fresh_ && control_interlocks_ok_ &&
        control_command_allows_scu &&
        BuildScuControlCommand(input, &command, &control_block_reason_);
    if (command_is_valid) {
      if (SendScuControl121(command)) {
        control_active_ = true;
        control_failsafe_active_ = false;
        last_active_gear_ = command.gear;
        control_block_reason_.clear();
      } else {
        control_block_reason_ = "UDP control send failed";
        if (control_active_) {
          StartFailsafe(now);
        }
      }
      return;
    }

    if (control_command_fresh_ && control_interlocks_ok_ &&
        !control_command_allows_scu) {
      control_active_ = false;
      control_failsafe_active_ = false;
      return;
    }

    if (state_.ccu.parking_status &&
        std::abs(state_.ccu.vehicle_speed_kph) <=
            kStationaryFailsafeSuppressSpeedKph) {
      control_active_ = false;
      control_failsafe_active_ = false;
      return;
    }

    if (!control_active_ && !control_failsafe_active_) {
      return;
    }
    StartFailsafe(now);
    if (AgeMs(true, failsafe_started_, now) > config_.failsafe_send_ms()) {
      control_active_ = false;
      control_failsafe_active_ = false;
      return;
    }

    YunleScuControlCommand failsafe;
    failsafe.drive_mode = YunleDriveModeRequest::kAutonomous;
    failsafe.gear = last_active_gear_;
    failsafe.target_speed_kph = 0.0;
    failsafe.front_steering_percentage = 0.0;
    failsafe.rear_steering_percentage = 0.0;
    failsafe.brake_enable = true;
    SendScuControl121(failsafe);
  }

  void FillWheelSpeed(Chassis* chassis) const {
    WheelSpeed* wheels = chassis->mutable_wheel_speed();
    const double meters_per_rpm =
        2.0 * kPi * config_.wheel_radius_m() / 60.0;

    if (config_.front_wheel_speed_valid()) {
      wheels->set_is_wheel_spd_fl_valid(true);
      wheels->set_wheel_spd_fl(state_.wheel_speed.front_left_rpm *
                               meters_per_rpm);
      wheels->set_wheel_direction_fl(
          ToWheelDirection(state_.wheel_speed.front_left_rpm));

      wheels->set_is_wheel_spd_fr_valid(true);
      wheels->set_wheel_spd_fr(state_.wheel_speed.front_right_rpm *
                               meters_per_rpm);
      wheels->set_wheel_direction_fr(
          ToWheelDirection(state_.wheel_speed.front_right_rpm));
    } else {
      wheels->set_is_wheel_spd_fl_valid(false);
      wheels->set_wheel_direction_fl(WheelSpeed::INVALID);
      wheels->set_is_wheel_spd_fr_valid(false);
      wheels->set_wheel_direction_fr(WheelSpeed::INVALID);
    }

    wheels->set_is_wheel_spd_rl_valid(true);
    wheels->set_wheel_spd_rl(state_.wheel_speed.rear_left_rpm *
                             meters_per_rpm);
    wheels->set_wheel_direction_rl(
        ToWheelDirection(state_.wheel_speed.rear_left_rpm));

    wheels->set_is_wheel_spd_rr_valid(true);
    wheels->set_wheel_spd_rr(state_.wheel_speed.rear_right_rpm *
                             meters_per_rpm);
    wheels->set_wheel_direction_rr(
        ToWheelDirection(state_.wheel_speed.rear_right_rpm));
  }

  void FillCcuStatus(Chassis* chassis) const {
    chassis->set_engine_started(state_.ccu.ignition_status != 0U);
    chassis->set_speed_mps(state_.ccu.vehicle_speed_kph / 3.6);
    chassis->set_parking_brake(state_.ccu.parking_status);
    chassis->set_gear_location(
        ToApolloGear(state_.ccu.shift_status, state_.ccu.parking_status));

    const double direction =
        state_.ccu.steering_direction_right ? -1.0 : 1.0;
    const double steering_percentage =
        state_.ccu.steering_magnitude /
        config_.max_steering_magnitude() * 100.0 * direction;
    chassis->set_steering_percentage(
        Clamp(steering_percentage, -100.0, 100.0));

    auto* signal = chassis->mutable_signal();
    if (state_.ccu.left_turn_light && state_.ccu.right_turn_light) {
      signal->set_turn_signal(common::VehicleSignal::TURN_HAZARD_WARNING);
    } else if (state_.ccu.left_turn_light) {
      signal->set_turn_signal(common::VehicleSignal::TURN_LEFT);
    } else if (state_.ccu.right_turn_light) {
      signal->set_turn_signal(common::VehicleSignal::TURN_RIGHT);
    } else {
      signal->set_turn_signal(common::VehicleSignal::TURN_NONE);
    }
    signal->set_low_beam(state_.ccu.low_beam);
  }

  void PublishChassis(const TimePoint& now) {
    auto chassis = std::make_shared<Chassis>();
    common::util::FillHeader("yunle_chassis_receiver", chassis.get());

    // The optional low-speed transmitter is an integration test path, not a
    // complete Apollo vehicle controller. It authorizes Apollo AUTO reporting
    // only when an explicit commissioning Profile enables it and the chassis
    // feedback itself is already in autonomous mode.
    const bool report_auto =
        config_.report_auto_driving_mode() && state_.ccu.drive_mode_raw == 1U &&
        state_.ccu.drive_mode_shift_button;
    chassis->set_driving_mode(report_auto ? Chassis::COMPLETE_AUTO_DRIVE
                                          : Chassis::COMPLETE_MANUAL);
    chassis->mutable_engage_advice()->set_advice(
        report_auto ? common::EngageAdvice::READY_TO_ENGAGE
                    : common::EngageAdvice::DISALLOW_ENGAGE);
    chassis->mutable_engage_advice()->set_reason(
        report_auto
            ? "Yunle commissioning Profile reports autonomous mode"
            : "Yunle chassis preview does not authorize autonomous engagement");

    const uint32_t timeout_ms = config_.feedback_timeout_ms();
    if (IsFresh(has_ccu_, last_ccu_, timeout_ms, now)) {
      FillCcuStatus(chassis.get());
    }
    if (IsFresh(has_wheel_speed_, last_wheel_speed_, timeout_ms, now)) {
      FillWheelSpeed(chassis.get());
    }
    if (IsFresh(has_bms_, last_bms_, config_.bms_timeout_ms(), now)) {
      const uint32_t soc = state_.bms.soc_percentage > 100U
                               ? 100U
                               : state_.bms.soc_percentage;
      chassis->set_battery_soc_percentage(static_cast<int32_t>(soc));
    }

    if (!CoreFeedbackIsFresh(now)) {
      chassis->set_error_code(Chassis::CHASSIS_CAN_LOST);
    } else if (state_.warning.MaxLevel() >= 2U) {
      chassis->set_error_code(Chassis::CHASSIS_ERROR);
    } else {
      chassis->set_error_code(Chassis::NO_ERROR);
    }
    if (IsFresh(has_warning_, last_warning_, timeout_ms, now)) {
      chassis->set_chassis_error_mask(
          static_cast<int32_t>(state_.warning.ErrorMask()));
    }

    if (!chassis_writer_->Write(chassis)) {
      AERROR_EVERY(100) << "Failed to publish Yunle Chassis.";
    }
  }

  void PublishDetail(const TimePoint& now) {
    auto detail = std::make_shared<YunleChassisDetail>();
    detail->set_timestamp_sec(cyber::Time::Now().ToSecond());
    detail->set_sequence_num(detail_sequence_++);
    detail->set_communication_ok(CoreFeedbackIsFresh(now));
    detail->set_received_frame_count(received_frame_count_);
    detail->set_recognized_frame_count(recognized_frame_count_);
    detail->set_invalid_packet_count(invalid_packet_count_);
    detail->set_unexpected_peer_count(unexpected_peer_count_);
    detail->set_last_info_byte(last_info_byte_);

    detail->set_ccu_age_ms(AgeMs(has_ccu_, last_ccu_, now));
    detail->set_wheel_speed_age_ms(
        AgeMs(has_wheel_speed_, last_wheel_speed_, now));
    detail->set_steering_age_ms(
        AgeMs(has_steering_, last_steering_, now));
    detail->set_warning_age_ms(AgeMs(has_warning_, last_warning_, now));
    detail->set_target_speed_age_ms(
        AgeMs(has_target_speed_, last_target_speed_, now));
    detail->set_bms_age_ms(AgeMs(has_bms_, last_bms_, now));

    if (has_ccu_) {
      detail->set_shift_status(state_.ccu.shift_status);
      detail->set_parking_status(state_.ccu.parking_status);
      detail->set_ignition_status(state_.ccu.ignition_status);
      detail->set_drive_mode_raw(state_.ccu.drive_mode_raw);
      detail->set_drive_mode_shift_button(
          state_.ccu.drive_mode_shift_button);
      detail->set_steering_direction_right(
          state_.ccu.steering_direction_right);
      detail->set_steering_magnitude(state_.ccu.steering_magnitude);
      detail->set_vehicle_speed_kph(state_.ccu.vehicle_speed_kph);
      detail->set_remote_brake(state_.ccu.remote_brake);
      detail->set_emergency_brake(state_.ccu.emergency_brake);
      detail->set_scu_brake(state_.ccu.scu_brake);
    }
    if (has_wheel_speed_) {
      detail->set_front_left_rpm(state_.wheel_speed.front_left_rpm);
      detail->set_front_right_rpm(state_.wheel_speed.front_right_rpm);
      detail->set_rear_left_rpm(state_.wheel_speed.rear_left_rpm);
      detail->set_rear_right_rpm(state_.wheel_speed.rear_right_rpm);
    }
    if (has_steering_) {
      detail->set_front_steering_angle_deg(state_.steering.front_angle_deg);
      detail->set_rear_steering_angle_deg(state_.steering.rear_angle_deg);
    }
    if (has_target_speed_) {
      detail->set_hardware_target_speed_kph(
          state_.target_speed.hardware_speed_kph);
      detail->set_scu_target_speed_kph(state_.target_speed.scu_speed_kph);
      detail->set_vehicle_target_speed_kph(
          state_.target_speed.vehicle_speed_kph);
      detail->set_real_speed_rpm(state_.target_speed.real_speed_rpm);
    }
    if (has_bms_) {
      detail->set_battery_voltage_v(state_.bms.voltage_v);
      detail->set_battery_current_a(state_.bms.current_a);
      detail->set_battery_soc_percentage(state_.bms.soc_percentage);
    }
    if (has_warning_) {
      detail->set_bms_charge_current_warning(
          state_.warning.bms_charge_current);
      detail->set_bms_discharge_current_warning(
          state_.warning.bms_discharge_current);
      detail->set_bms_soc_warning(state_.warning.bms_soc);
      detail->set_bms_temperature_warning(state_.warning.bms_temperature);
      detail->set_mcu_current_warning(state_.warning.mcu_current);
      detail->set_mcu_disconnect_warning(state_.warning.mcu_disconnect);
      detail->set_mcu_motor_warning(state_.warning.mcu_motor);
      detail->set_mcu_speed_warning(state_.warning.mcu_speed);
      detail->set_mcu_temperature_warning(state_.warning.mcu_temperature);
      detail->set_mcu_voltage_warning(state_.warning.mcu_voltage);
      detail->set_steering_disconnect_warning(
          state_.warning.steering_disconnect);
      detail->set_steering_lock_warning(state_.warning.steering_lock);
      detail->set_steering_unstoppable_warning(
          state_.warning.steering_unstoppable);
      detail->set_max_warning_level(state_.warning.MaxLevel());
    }

    detail->set_control_send_enabled(config_.enable_control_send());
    detail->set_control_command_received(control_command_received_);
    detail->set_control_command_fresh(control_command_fresh_);
    detail->set_control_interlocks_ok(control_interlocks_ok_);
    detail->set_control_failsafe_active(control_failsafe_active_);
    detail->set_sent_control_frame_count(sent_control_frame_count_);
    detail->set_control_send_error_count(control_send_error_count_);
    detail->set_commanded_target_speed_kph(commanded_target_speed_kph_);
    detail->set_commanded_steering_percentage(
        commanded_steering_percentage_);
    detail->set_commanded_gear(commanded_gear_);
    detail->set_control_block_reason(control_block_reason_);

    if (!detail_writer_->Write(detail)) {
      AERROR_EVERY(100) << "Failed to publish Yunle chassis detail.";
    }
  }

  YunleChassisReceiverConfig config_;
  int socket_fd_ = -1;
  in_addr expected_remote_address_ {};
  uint16_t expected_remote_port_ = 0;
  sockaddr_in remote_address_ {};
  std::array<uint8_t, 65535> receive_buffer_ {};

  std::shared_ptr<cyber::Writer<Chassis>> chassis_writer_;
  std::shared_ptr<cyber::Writer<YunleChassisDetail>> detail_writer_;
  std::shared_ptr<cyber::Reader<control::ControlCommand>> control_reader_;

  std::mutex control_command_mutex_;
  control::ControlCommand latest_control_command_;
  bool has_control_command_ = false;
  TimePoint last_control_command_ {};

  bool control_command_received_ = false;
  bool control_command_fresh_ = false;
  bool control_interlocks_ok_ = false;
  bool control_active_ = false;
  bool control_failsafe_active_ = false;
  TimePoint failsafe_started_ {};
  YunleGearRequest last_active_gear_ = YunleGearRequest::kNeutral;
  uint64_t sent_control_frame_count_ = 0;
  uint64_t control_send_error_count_ = 0;
  double commanded_target_speed_kph_ = 0.0;
  double commanded_steering_percentage_ = 0.0;
  uint32_t commanded_gear_ = 0U;
  std::string control_block_reason_ = "control send disabled by Profile";

  YunleChassisState state_;
  bool has_ccu_ = false;
  bool has_wheel_speed_ = false;
  bool has_steering_ = false;
  bool has_warning_ = false;
  bool has_target_speed_ = false;
  bool has_bms_ = false;
  TimePoint last_ccu_ {};
  TimePoint last_wheel_speed_ {};
  TimePoint last_steering_ {};
  TimePoint last_warning_ {};
  TimePoint last_target_speed_ {};
  TimePoint last_bms_ {};

  uint64_t received_frame_count_ = 0;
  uint64_t recognized_frame_count_ = 0;
  uint64_t invalid_packet_count_ = 0;
  uint64_t unexpected_peer_count_ = 0;
  uint64_t detail_sequence_ = 0;
  uint8_t last_info_byte_ = 0;
};

CYBER_REGISTER_COMPONENT(YunleChassisReceiverComponent)

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
