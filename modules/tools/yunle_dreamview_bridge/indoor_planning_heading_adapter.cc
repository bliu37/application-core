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

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kLocalizationTopic[] = "/apollo/localization/pose";
constexpr char kRoutingResponseTopic[] = "/apollo/routing_response";
constexpr char kPlanningLocalizationTopic[] =
    "/apollo/yunle/indoor/planning/localization_pose";
constexpr double kPi = 3.14159265358979323846;

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

bool RouteHeadingFromRequest(const routing::RoutingRequest& request,
                             double* heading) {
  if (request.waypoint_size() < 2 || heading == nullptr) {
    return false;
  }
  const auto& start = request.waypoint(0);
  const auto& end = request.waypoint(request.waypoint_size() - 1);
  if (start.has_heading()) {
    *heading = start.heading();
    return std::isfinite(*heading);
  }
  if (!start.has_pose() || !end.has_pose()) {
    return false;
  }
  const double dx = end.pose().x() - start.pose().x();
  const double dy = end.pose().y() - start.pose().y();
  if (std::hypot(dx, dy) < 1e-3) {
    return false;
  }
  *heading = std::atan2(dy, dx);
  return std::isfinite(*heading);
}

}  // namespace

// Publishes a Planning-only localization stream whose X/Y/Z remains the
// stabilized indoor NDT pose, while heading follows the active route direction.
// The raw Apollo localization heading is already corrected by the stabilizer;
// this route-specific stream keeps Planning aligned to the lane direction after
// Dreamview sends a route.
class YunleIndoorPlanningHeadingAdapter final : public cyber::Component<> {
 public:
  bool Init() override {
    localization_writer_ =
        node_->CreateWriter<localization::LocalizationEstimate>(
            kPlanningLocalizationTopic);
    if (localization_writer_ == nullptr) {
      AERROR << "Failed to create indoor Planning localization writer.";
      return false;
    }

    localization_reader_ =
        node_->CreateReader<localization::LocalizationEstimate>(
            kLocalizationTopic,
            [this](const std::shared_ptr<localization::LocalizationEstimate>&
                       msg) { OnLocalization(msg); });
    routing_reader_ = node_->CreateReader<routing::RoutingResponse>(
        kRoutingResponseTopic,
        [this](const std::shared_ptr<routing::RoutingResponse>& msg) {
          OnRoutingResponse(msg);
        });
    if (localization_reader_ == nullptr || routing_reader_ == nullptr) {
      AERROR << "Failed to create indoor Planning heading adapter readers.";
      return false;
    }

    AINFO << "Yunle indoor Planning heading adapter started. Input localization="
          << kLocalizationTopic << ", routing=" << kRoutingResponseTopic
          << ", output=" << kPlanningLocalizationTopic << ".";
    return true;
  }

 private:
  void OnRoutingResponse(const std::shared_ptr<routing::RoutingResponse>& msg) {
    if (msg == nullptr || !msg->has_routing_request()) {
      return;
    }
    double heading = 0.0;
    if (!RouteHeadingFromRequest(msg->routing_request(), &heading)) {
      AWARN_EVERY(20) << "RoutingResponse has no usable route heading.";
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      route_heading_ = heading;
      has_route_heading_ = true;
    }
    AINFO << "Updated indoor Planning route heading: " << heading
          << " rad (" << heading * 180.0 / kPi << " deg).";
  }

  void OnLocalization(
      const std::shared_ptr<localization::LocalizationEstimate>& msg) {
    if (msg == nullptr || !msg->has_pose() || !IsFinitePose(msg->pose())) {
      AWARN_EVERY(50) << "Ignoring invalid localization for Planning adapter.";
      return;
    }

    double heading = 0.0;
    bool has_heading = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      heading = route_heading_;
      has_heading = has_route_heading_;
    }

    auto output =
        std::make_shared<localization::LocalizationEstimate>(*msg);
    auto* pose = output->mutable_pose();
    if (has_heading) {
      Rpy rpy = QuaternionToRpy(pose->orientation());
      rpy.yaw = heading;
      SetQuaternionFromRpy(rpy, pose->mutable_orientation());
      pose->set_heading(heading);
      pose->mutable_euler_angles()->set_x(rpy.pitch);
      pose->mutable_euler_angles()->set_y(rpy.roll);
      pose->mutable_euler_angles()->set_z(rpy.yaw);
    }

    AINFO_EVERY(100) << "Planning localization published: xy=("
                     << pose->position().x() << ", " << pose->position().y()
                     << ") heading=" << pose->heading()
                     << " route_heading_valid=" << has_heading;
    localization_writer_->Write(output);
  }

  std::mutex mutex_;
  double route_heading_ = 0.0;
  bool has_route_heading_ = false;

  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      localization_writer_;
};

CYBER_REGISTER_COMPONENT(YunleIndoorPlanningHeadingAdapter)

}  // namespace yunle
}  // namespace apollo
