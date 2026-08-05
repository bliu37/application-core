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
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/loam_velodyne_indoor/proto/odometry.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kRawPointCloudTopic[] =
    "/apollo/sensor/lslidar16v4/PointCloud2";
constexpr char kLiorfOdometryTopic[] = "liorf/mapping/odometry";
constexpr char kNdtOdometryTopic[] = "/apollo/yunle/indoor/ndt/odometry";
constexpr char kNdtPointCloudTopic[] =
    "/apollo/yunle/indoor/ndt/PointCloud2";

// The saved LIORF map uses this artificial indoor origin. It is recorded in
// jd03_indoor_20260803_01/gnss-map-offset.txt and intentionally keeps the
// fixed-map coordinates away from NDT's rejected all-zero odometry pose.
constexpr double kMapOffsetX = 10000.0;
constexpr double kMapOffsetY = 9000000.0;
constexpr double kMapOffsetZ = 0.0;
constexpr double kFloatSafeMapOffsetY = 10000.0;

constexpr std::size_t kMaxCloudCacheSize = 30;
constexpr double kMaxCloudOdomTimeDifferenceSec = 0.35;
constexpr double kNdtInterpolationMarginSec = 0.001;
double PointCloudTimeSec(const drivers::PointCloud& cloud) {
  // PointCloud.measurement_time and Header.timestamp_sec are both expressed
  // in seconds. Prefer the header because it is also the timestamp consumed
  // by LIORF and therefore provides the clearest pairing reference.
  const double header_time_sec = cloud.header().timestamp_sec();
  if (std::isfinite(header_time_sec) && header_time_sec > 0.0) {
    return header_time_sec;
  }
  return cloud.measurement_time();
}

bool IsFinitePose(const localization::Pose& pose) {
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

}  // namespace

// Adapts the proven indoor LIORF odometry to the relative-pose input expected
// by Apollo's CPU NDT localizer. It also republishes the corresponding raw
// point cloud one lidar frame later. The deliberate delay lets NDT receive two
// odometry samples bracketing the cloud timestamp before it interpolates the
// initial pose.
//
// This is a localization bring-up adapter. LIORF remains the relative motion
// source; NDT supplies the correction against the previously saved fixed map.
class YunleIndoorNdtInputBridge : public cyber::Component<> {
 public:
  bool Init() override {
    ndt_odometry_writer_ =
        node_->CreateWriter<localization::Gps>(kNdtOdometryTopic);
    ndt_point_cloud_writer_ =
        node_->CreateWriter<drivers::PointCloud>(kNdtPointCloudTopic);
    if (ndt_odometry_writer_ == nullptr || ndt_point_cloud_writer_ == nullptr) {
      AERROR << "Failed to create Yunle indoor NDT bridge writers.";
      return false;
    }

    raw_point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
        kRawPointCloudTopic,
        [this](const std::shared_ptr<drivers::PointCloud>& cloud) {
          OnPointCloud(cloud);
        });
    liorf_odometry_reader_ =
        node_->CreateReader<loam_velodyne_indoor::Odometry>(
            kLiorfOdometryTopic,
            [this](const std::shared_ptr<loam_velodyne_indoor::Odometry>&
                       odometry) { OnOdometry(odometry); });
    if (raw_point_cloud_reader_ == nullptr ||
        liorf_odometry_reader_ == nullptr) {
      AERROR << "Failed to create Yunle indoor NDT bridge readers.";
      return false;
    }

    AINFO << "Yunle indoor NDT input bridge started. LIORF odometry is "
          << "translated by (" << kMapOffsetX << ", " << MapOffsetY() << ", "
          << kMapOffsetZ << ") and paired with delayed LS-C16 V4 clouds.";
    return true;
  }

 protected:
  virtual double MapOffsetY() const { return kMapOffsetY; }

 private:
  void OnPointCloud(const std::shared_ptr<drivers::PointCloud>& cloud) {
    if (cloud == nullptr || cloud->point_size() == 0) {
      return;
    }

    std::shared_ptr<drivers::PointCloud> cloud_to_publish;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      point_cloud_cache_.push_back(cloud);
      while (point_cloud_cache_.size() > kMaxCloudCacheSize) {
        point_cloud_cache_.pop_front();
      }

      // Publishing the cloud selected by the previous odometry callback now
      // gives NDT roughly one lidar period to receive the odometry first.
      cloud_to_publish = pending_point_cloud_;
      pending_point_cloud_.reset();
    }

    if (cloud_to_publish != nullptr) {
      ndt_point_cloud_writer_->Write(cloud_to_publish);
    }
  }

  void OnOdometry(
      const std::shared_ptr<loam_velodyne_indoor::Odometry>& odometry) {
    if (odometry == nullptr || !odometry->has_header() ||
        !odometry->has_pose() || !odometry->pose().has_pose()) {
      AWARN_EVERY(50) << "Ignoring incomplete LIORF odometry.";
      return;
    }

    const double timestamp_sec = odometry->header().timestamp_sec();
    const auto& source_pose = odometry->pose().pose();
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0 ||
        !IsFinitePose(source_pose)) {
      AWARN_EVERY(50) << "Ignoring invalid LIORF odometry pose.";
      return;
    }

    auto gps = std::make_shared<localization::Gps>();
    gps->mutable_header()->CopyFrom(odometry->header());
    gps->mutable_header()->set_module_name(node_->Name());
    gps->mutable_header()->set_timestamp_sec(timestamp_sec);
    gps->mutable_localization()->CopyFrom(source_pose);
    auto* position = gps->mutable_localization()->mutable_position();
    position->set_x(position->x() + kMapOffsetX);
    position->set_y(position->y() + MapOffsetY());
    position->set_z(position->z() + kMapOffsetZ);

    if (odometry->has_twist() && odometry->twist().has_twist()) {
      const auto& twist = odometry->twist().twist();
      if (twist.has_linear()) {
        auto* velocity = gps->mutable_localization()->mutable_linear_velocity();
        velocity->set_x(twist.linear().x());
        velocity->set_y(twist.linear().y());
        velocity->set_z(twist.linear().z());
      }
      if (twist.has_angular()) {
        auto* angular =
            gps->mutable_localization()->mutable_angular_velocity();
        angular->set_x(twist.angular().x());
        angular->set_y(twist.angular().y());
        angular->set_z(twist.angular().z());
        auto* angular_vrf =
            gps->mutable_localization()->mutable_angular_velocity_vrf();
        angular_vrf->set_x(twist.angular().x());
        angular_vrf->set_y(twist.angular().y());
        angular_vrf->set_z(twist.angular().z());
      }
    }
    auto* acceleration =
        gps->mutable_localization()->mutable_linear_acceleration();
    acceleration->set_x(0.0);
    acceleration->set_y(0.0);
    acceleration->set_z(0.0);
    auto* acceleration_vrf =
        gps->mutable_localization()->mutable_linear_acceleration_vrf();
    acceleration_vrf->set_x(0.0);
    acceleration_vrf->set_y(0.0);
    acceleration_vrf->set_z(0.0);
    ndt_odometry_writer_->Write(gps);

    std::shared_ptr<drivers::PointCloud> closest_cloud;
    double closest_difference = std::numeric_limits<double>::max();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& cloud : point_cloud_cache_) {
        const double difference =
            std::abs(PointCloudTimeSec(*cloud) - timestamp_sec);
        if (difference < closest_difference) {
          closest_difference = difference;
          closest_cloud = cloud;
        }
      }

      if (closest_cloud != nullptr &&
          closest_difference <= kMaxCloudOdomTimeDifferenceSec) {
        pending_point_cloud_ =
            std::make_shared<drivers::PointCloud>(*closest_cloud);
        const double interpolation_time =
            timestamp_sec - kNdtInterpolationMarginSec;
        // PointCloud.measurement_time is a seconds-valued double. NDT passes
        // it to cyber::Time(double), whose constructor also expects seconds.
        pending_point_cloud_->set_measurement_time(interpolation_time);
      }
    }

    if (closest_cloud == nullptr ||
        closest_difference > kMaxCloudOdomTimeDifferenceSec) {
      AWARN_EVERY(20) << "No LS-C16 V4 cloud close enough to LIORF odometry; "
                      << "difference=" << closest_difference << " s.";
    }
  }

  std::shared_ptr<cyber::Reader<drivers::PointCloud>> raw_point_cloud_reader_;
  std::shared_ptr<cyber::Reader<loam_velodyne_indoor::Odometry>>
      liorf_odometry_reader_;
  std::shared_ptr<cyber::Writer<localization::Gps>> ndt_odometry_writer_;
  std::shared_ptr<cyber::Writer<drivers::PointCloud>>
      ndt_point_cloud_writer_;

  std::mutex mutex_;
  std::deque<std::shared_ptr<drivers::PointCloud>> point_cloud_cache_;
  std::shared_ptr<drivers::PointCloud> pending_point_cloud_;
};

class YunleIndoorNdtFloatSafeInputBridge final
    : public YunleIndoorNdtInputBridge {
 protected:
  double MapOffsetY() const override { return kFloatSafeMapOffsetY; }
};

CYBER_REGISTER_COMPONENT(YunleIndoorNdtInputBridge)
CYBER_REGISTER_COMPONENT(YunleIndoorNdtFloatSafeInputBridge)

}  // namespace yunle
}  // namespace apollo
