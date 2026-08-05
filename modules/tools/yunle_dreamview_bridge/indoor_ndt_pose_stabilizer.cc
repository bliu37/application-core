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

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/common_msgs/localization_msgs/gps.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kRawLocalizationTopic[] = "/apollo/yunle/indoor/ndt/raw_pose";
constexpr char kRawNdtLidarTopic[] = "/apollo/yunle/indoor/ndt/raw_lidar";
constexpr char kNdtOdometryTopic[] = "/apollo/yunle/indoor/ndt/odometry";
constexpr char kLocalizationTopic[] = "/apollo/localization/pose";
constexpr char kNdtLidarTopic[] = "/apollo/localization/ndt_lidar";

constexpr std::size_t kMaxOdometryCacheSize = 300;
constexpr double kMaxOdometryTimeDifferenceSec = 0.50;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxNdtPlanarCorrectionM = 0.50;
constexpr double kMaxNdtYawCorrectionRad = 10.0 * kPi / 180.0;
constexpr double kApolloVehicleHeadingOffsetRad = kPi / 2.0;

struct Rpy {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

template <typename Quaternion>
Rpy QuaternionToRpy(const Quaternion& q) {
  Rpy rpy;
  const double qx = q.qx();
  const double qy = q.qy();
  const double qz = q.qz();
  const double qw = q.qw();

  const double sin_roll = 2.0 * (qw * qx + qy * qz);
  const double cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy);
  rpy.roll = std::atan2(sin_roll, cos_roll);

  const double sin_pitch = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sin_pitch) >= 1.0) {
    rpy.pitch = std::copysign(kPi / 2.0, sin_pitch);
  } else {
    rpy.pitch = std::asin(sin_pitch);
  }

  const double sin_yaw = 2.0 * (qw * qz + qx * qy);
  const double cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz);
  rpy.yaw = std::atan2(sin_yaw, cos_yaw);
  return rpy;
}

template <typename Quaternion>
void SetQuaternionFromRpy(const Rpy& rpy, Quaternion* q) {
  const double half_roll = rpy.roll * 0.5;
  const double half_pitch = rpy.pitch * 0.5;
  const double half_yaw = rpy.yaw * 0.5;
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  double qw = cr * cp * cy + sr * sp * sy;
  double qx = sr * cp * cy - cr * sp * sy;
  double qy = cr * sp * cy + sr * cp * sy;
  double qz = cr * cp * sy - sr * sp * cy;
  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm > 0.0) {
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
  }
  q->set_qw(qw);
  q->set_qx(qx);
  q->set_qy(qy);
  q->set_qz(qz);
}

template <typename Pose>
bool IsFinitePose(const Pose& pose) {
  if (!pose.has_position() || !pose.has_orientation()) {
    return false;
  }
  const auto& p = pose.position();
  const auto& q = pose.orientation();
  const double quaternion_norm =
      std::sqrt(q.qx() * q.qx() + q.qy() * q.qy() + q.qz() * q.qz() +
                q.qw() * q.qw());
  return std::isfinite(p.x()) && std::isfinite(p.y()) &&
         std::isfinite(p.z()) && std::isfinite(q.qx()) &&
         std::isfinite(q.qy()) && std::isfinite(q.qz()) &&
         std::isfinite(q.qw()) && quaternion_norm > 0.5;
}

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double ApolloVehicleYawFromIndoorNdtYaw(double yaw) {
  return NormalizeAngle(yaw + kApolloVehicleHeadingOffsetRad);
}

}  // namespace

// Stabilizes Apollo CPU-NDT/PCL-OMP-NDT's indoor output against LIORF's motion
// prior.
//
// The current JD03 indoor map can let Apollo's 6-DoF CPU-NDT converge to
// stable but wrong local optima while the vehicle is stationary. Keep small
// map-frame NDT corrections, but gate large X/Y/yaw jumps and fall back to the
// LIORF odometry pose that is already feeding NDT as its motion prior. The
// LIORF/NDT yaw convention is 90 degrees clockwise from Apollo's vehicle
// heading, so final Apollo localization adds +90 degrees while preserving the
// corrected map-frame X/Y.
class YunleIndoorNdtPoseStabilizer final : public cyber::Component<> {
 public:
  bool Init() override {
    localization_writer_ =
        node_->CreateWriter<localization::LocalizationEstimate>(
            kLocalizationTopic);
    ndt_lidar_writer_ =
        node_->CreateWriter<localization::LocalizationEstimate>(kNdtLidarTopic);
    if (localization_writer_ == nullptr || ndt_lidar_writer_ == nullptr) {
      AERROR << "Failed to create Yunle indoor NDT stabilized pose writers.";
      return false;
    }

    odometry_reader_ = node_->CreateReader<localization::Gps>(
        kNdtOdometryTopic,
        [this](const std::shared_ptr<localization::Gps>& odometry) {
          OnOdometry(odometry);
        });
    raw_localization_reader_ =
        node_->CreateReader<localization::LocalizationEstimate>(
            kRawLocalizationTopic,
            [this](
                const std::shared_ptr<localization::LocalizationEstimate>& msg) {
              StabilizeAndPublish(msg, localization_writer_,
                                  kRawLocalizationTopic, kLocalizationTopic);
            });
    raw_ndt_lidar_reader_ =
        node_->CreateReader<localization::LocalizationEstimate>(
            kRawNdtLidarTopic,
            [this](
                const std::shared_ptr<localization::LocalizationEstimate>& msg) {
              StabilizeAndPublish(msg, ndt_lidar_writer_, kRawNdtLidarTopic,
                                  kNdtLidarTopic);
            });
    if (odometry_reader_ == nullptr || raw_localization_reader_ == nullptr ||
        raw_ndt_lidar_reader_ == nullptr) {
      AERROR << "Failed to create Yunle indoor NDT stabilizer readers.";
      return false;
    }

    AINFO << "Yunle indoor NDT pose stabilizer started. Raw CPU-NDT topics are "
          << kRawLocalizationTopic << " and " << kRawNdtLidarTopic
          << "; stabilized outputs are " << kLocalizationTopic << " and "
          << kNdtLidarTopic << ".";
    return true;
  }

 private:
  void OnOdometry(const std::shared_ptr<localization::Gps>& odometry) {
    if (odometry == nullptr || !odometry->has_header() ||
        !odometry->has_localization() ||
        !IsFinitePose(odometry->localization())) {
      AWARN_EVERY(50) << "Ignoring incomplete NDT odometry prior.";
      return;
    }
    const double timestamp_sec = odometry->header().timestamp_sec();
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0) {
      AWARN_EVERY(50) << "Ignoring NDT odometry prior with invalid time.";
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    odometry_cache_.push_back(odometry);
    while (odometry_cache_.size() > kMaxOdometryCacheSize) {
      odometry_cache_.pop_front();
    }
  }

  bool FindNearestOdometry(
      double timestamp_sec,
      std::shared_ptr<localization::Gps>* nearest_odometry) const {
    double nearest_difference = std::numeric_limits<double>::max();
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& odometry : odometry_cache_) {
      const double difference =
          std::abs(odometry->header().timestamp_sec() - timestamp_sec);
      if (difference < nearest_difference) {
        nearest_difference = difference;
        *nearest_odometry = odometry;
      }
    }
    return *nearest_odometry != nullptr &&
           nearest_difference <= kMaxOdometryTimeDifferenceSec;
  }

  void StabilizeAndPublish(
      const std::shared_ptr<localization::LocalizationEstimate>& raw_msg,
      const std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>&
          writer,
      const char* input_topic, const char* output_topic) {
    if (raw_msg == nullptr || !raw_msg->has_pose() ||
        !IsFinitePose(raw_msg->pose())) {
      AWARN_EVERY(50) << "Ignoring incomplete raw NDT localization from "
                      << input_topic;
      return;
    }
    const double timestamp_sec = raw_msg->measurement_time();
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0) {
      AWARN_EVERY(50) << "Ignoring raw NDT localization with invalid time from "
                      << input_topic;
      return;
    }

    std::shared_ptr<localization::Gps> odometry;
    if (!FindNearestOdometry(timestamp_sec, &odometry)) {
      AWARN_EVERY(50) << "No nearby NDT odometry prior for raw localization from "
                      << input_topic << "; skip stabilized output.";
      return;
    }

    auto output =
        std::make_shared<localization::LocalizationEstimate>(*raw_msg);
    const auto& raw_pose = raw_msg->pose();
    const auto& odometry_pose = odometry->localization();
    auto* pose = output->mutable_pose();

    const Rpy raw_rpy = QuaternionToRpy(raw_pose.orientation());
    const Rpy odometry_rpy = QuaternionToRpy(odometry_pose.orientation());
    const double correction_x =
        raw_pose.position().x() - odometry_pose.position().x();
    const double correction_y =
        raw_pose.position().y() - odometry_pose.position().y();
    const double planar_correction =
        std::hypot(correction_x, correction_y);
    const double yaw_correction =
        std::abs(NormalizeAngle(raw_rpy.yaw - odometry_rpy.yaw));
    const bool accept_ndt_correction =
        planar_correction <= kMaxNdtPlanarCorrectionM &&
        yaw_correction <= kMaxNdtYawCorrectionRad;

    auto* position = pose->mutable_position();
    if (!accept_ndt_correction) {
      position->set_x(odometry_pose.position().x());
      position->set_y(odometry_pose.position().y());
    }
    position->set_z(odometry_pose.position().z());

    Rpy stabilized_rpy;
    stabilized_rpy.roll = odometry_rpy.roll;
    stabilized_rpy.pitch = odometry_rpy.pitch;
    const double ndt_yaw =
        accept_ndt_correction ? raw_rpy.yaw : odometry_rpy.yaw;
    stabilized_rpy.yaw = ApolloVehicleYawFromIndoorNdtYaw(ndt_yaw);
    SetQuaternionFromRpy(stabilized_rpy, pose->mutable_orientation());
    pose->set_heading(stabilized_rpy.yaw);
    pose->mutable_euler_angles()->set_x(stabilized_rpy.pitch);
    pose->mutable_euler_angles()->set_y(stabilized_rpy.roll);
    pose->mutable_euler_angles()->set_z(stabilized_rpy.yaw);

    if (odometry_pose.has_angular_velocity_vrf()) {
      pose->mutable_angular_velocity_vrf()->CopyFrom(
          odometry_pose.angular_velocity_vrf());
    } else if (odometry_pose.has_angular_velocity()) {
      pose->mutable_angular_velocity_vrf()->CopyFrom(
          odometry_pose.angular_velocity());
    } else if (!pose->has_angular_velocity_vrf()) {
      auto* angular_vrf = pose->mutable_angular_velocity_vrf();
      angular_vrf->set_x(0.0);
      angular_vrf->set_y(0.0);
      angular_vrf->set_z(0.0);
    }
    if (odometry_pose.has_linear_acceleration_vrf()) {
      pose->mutable_linear_acceleration_vrf()->CopyFrom(
          odometry_pose.linear_acceleration_vrf());
    } else if (odometry_pose.has_linear_acceleration()) {
      pose->mutable_linear_acceleration_vrf()->CopyFrom(
          odometry_pose.linear_acceleration());
    } else if (!pose->has_linear_acceleration_vrf()) {
      auto* acceleration_vrf = pose->mutable_linear_acceleration_vrf();
      acceleration_vrf->set_x(0.0);
      acceleration_vrf->set_y(0.0);
      acceleration_vrf->set_z(0.0);
    }
    if (!pose->has_linear_acceleration()) {
      pose->mutable_linear_acceleration()->CopyFrom(
          pose->linear_acceleration_vrf());
    }
    if (!pose->has_angular_velocity()) {
      pose->mutable_angular_velocity()->CopyFrom(pose->angular_velocity_vrf());
    }

    AINFO_EVERY(100) << "Stabilized " << input_topic << " -> " << output_topic
                     << ": xy=(" << raw_pose.position().x() << ", "
                     << raw_pose.position().y() << ") correction=("
                     << correction_x << ", " << correction_y
                     << ") planar=" << planar_correction << " yaw_deg="
                     << yaw_correction * 180.0 / kPi << " accepted="
                     << accept_ndt_correction << " z "
                     << raw_pose.position().z() << " -> "
                     << odometry_pose.position().z() << " ndt_yaw_deg="
                     << ndt_yaw * 180.0 / kPi << " apollo_heading_deg="
                     << stabilized_rpy.yaw * 180.0 / kPi;
    writer->Write(output);
  }

  mutable std::mutex mutex_;
  std::deque<std::shared_ptr<localization::Gps>> odometry_cache_;

  std::shared_ptr<cyber::Reader<localization::Gps>> odometry_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      raw_localization_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      raw_ndt_lidar_reader_;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      localization_writer_;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      ndt_lidar_writer_;
};

CYBER_REGISTER_COMPONENT(YunleIndoorNdtPoseStabilizer)

}  // namespace yunle
}  // namespace apollo
