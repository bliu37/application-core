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
#include <string>

#include "pcl/common/transforms.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pclomp/ndt_omp.h"
#include "yaml-cpp/yaml.h"

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/common_msgs/localization_msgs/gps.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace yunle {

namespace {

using PclPoint = pcl::PointXYZI;
using PclCloud = pcl::PointCloud<PclPoint>;

constexpr char kNdtPointCloudTopic[] =
    "/apollo/yunle/indoor/ndt/PointCloud2";
constexpr char kNdtOdometryTopic[] = "/apollo/yunle/indoor/ndt/odometry";
constexpr char kRawLocalizationTopic[] = "/apollo/yunle/indoor/ndt/raw_pose";
constexpr char kRawNdtLidarTopic[] = "/apollo/yunle/indoor/ndt/raw_lidar";

constexpr char kMapPcdPath[] =
    "/apollo_workspace/data/map_work/yunle_indoor/"
    "jd03_indoor_20260803_03/global.pcd";
constexpr char kLidarExtrinsicsPath[] =
    "/apollo/modules/localization/msf/params/"
    "yunle_indoor_ndt_lidar_extrinsics_minus_90_measured.yaml";

constexpr double kMapOffsetX = 10000.0;
constexpr double kMapOffsetY = 10000.0;
constexpr double kMapOffsetZ = 0.0;
constexpr double kPi = 3.14159265358979323846;

constexpr double kMaxOdometryTimeDifferenceSec = 0.50;
constexpr std::size_t kMaxOdometryCacheSize = 300;
constexpr double kMaxScanRangeM = 70.0;
constexpr float kMapVoxelLeafM = 0.20f;
constexpr float kScanVoxelLeafM = 0.30f;
constexpr float kScanVoxelLeafZ = 0.10f;
constexpr float kMinScanZ = -0.15f;
constexpr float kMaxScanZ = 20.0f;
constexpr std::size_t kMinFilteredScanPoints = 30;

constexpr double kNdtTransformationEpsilon = 0.01;
constexpr double kNdtStepSize = 0.05;
constexpr float kNdtResolution = 1.5f;
constexpr int kNdtMaxIterations = 35;
constexpr int kNdtNumThreads = 5;
constexpr double kMinTransformProbability = 3.0;

struct Rpy {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

double PointCloudTimeSec(const drivers::PointCloud& cloud) {
  if (std::isfinite(cloud.measurement_time()) && cloud.measurement_time() > 0.0) {
    return cloud.measurement_time();
  }
  if (cloud.has_header() && std::isfinite(cloud.header().timestamp_sec()) &&
      cloud.header().timestamp_sec() > 0.0) {
    return cloud.header().timestamp_sec();
  }
  return 0.0;
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

Eigen::Affine3d PoseToAffine(const localization::Pose& pose) {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.translation().x() = pose.position().x();
  affine.translation().y() = pose.position().y();
  affine.translation().z() = pose.position().z();

  Eigen::Quaterniond quat(pose.orientation().qw(), pose.orientation().qx(),
                          pose.orientation().qy(), pose.orientation().qz());
  quat.normalize();
  affine.linear() = quat.toRotationMatrix();
  return affine;
}

Rpy QuaternionToRpy(const Eigen::Quaterniond& quat) {
  const double qx = quat.x();
  const double qy = quat.y();
  const double qz = quat.z();
  const double qw = quat.w();

  Rpy rpy;
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

void FillPose(const Eigen::Affine3d& affine, localization::Pose* pose) {
  auto* position = pose->mutable_position();
  position->set_x(affine.translation().x());
  position->set_y(affine.translation().y());
  position->set_z(affine.translation().z());

  Eigen::Quaterniond quat(affine.linear());
  quat.normalize();
  auto* orientation = pose->mutable_orientation();
  orientation->set_qw(quat.w());
  orientation->set_qx(quat.x());
  orientation->set_qy(quat.y());
  orientation->set_qz(quat.z());

  const Rpy rpy = QuaternionToRpy(quat);
  pose->set_heading(rpy.yaw);
  pose->mutable_euler_angles()->set_x(rpy.pitch);
  pose->mutable_euler_angles()->set_y(rpy.roll);
  pose->mutable_euler_angles()->set_z(rpy.yaw);
}

void CopyMotionFields(const localization::Pose& from, localization::Pose* to) {
  if (from.has_linear_velocity()) {
    to->mutable_linear_velocity()->CopyFrom(from.linear_velocity());
  }
  if (from.has_linear_acceleration()) {
    to->mutable_linear_acceleration()->CopyFrom(from.linear_acceleration());
  }
  if (from.has_angular_velocity()) {
    to->mutable_angular_velocity()->CopyFrom(from.angular_velocity());
  }
  if (from.has_linear_acceleration_vrf()) {
    to->mutable_linear_acceleration_vrf()->CopyFrom(
        from.linear_acceleration_vrf());
  }
  if (from.has_angular_velocity_vrf()) {
    to->mutable_angular_velocity_vrf()->CopyFrom(from.angular_velocity_vrf());
  }
}

bool LoadLidarExtrinsic(const std::string& file_path,
                        Eigen::Affine3d* lidar_extrinsic) {
  if (lidar_extrinsic == nullptr) {
    return false;
  }

  YAML::Node config = YAML::LoadFile(file_path);
  if (!config["transform"] || !config["transform"]["translation"] ||
      !config["transform"]["rotation"]) {
    return false;
  }
  const auto translation = config["transform"]["translation"];
  const auto rotation = config["transform"]["rotation"];

  lidar_extrinsic->setIdentity();
  lidar_extrinsic->translation().x() = translation["x"].as<double>();
  lidar_extrinsic->translation().y() = translation["y"].as<double>();
  lidar_extrinsic->translation().z() = translation["z"].as<double>();
  Eigen::Quaterniond quat(rotation["w"].as<double>(), rotation["x"].as<double>(),
                          rotation["y"].as<double>(),
                          rotation["z"].as<double>());
  quat.normalize();
  lidar_extrinsic->linear() = quat.toRotationMatrix();
  return true;
}

}  // namespace

class YunleIndoorPclOmpNdtLocalizer final : public cyber::Component<> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool Init() override {
    raw_localization_writer_ =
        node_->CreateWriter<localization::LocalizationEstimate>(
            kRawLocalizationTopic);
    raw_ndt_lidar_writer_ =
        node_->CreateWriter<localization::LocalizationEstimate>(
            kRawNdtLidarTopic);
    if (raw_localization_writer_ == nullptr ||
        raw_ndt_lidar_writer_ == nullptr) {
      AERROR << "Failed to create Yunle PCL-OMP NDT raw localization writers.";
      return false;
    }

    if (!LoadLidarExtrinsic(kLidarExtrinsicsPath, &lidar_extrinsic_)) {
      AERROR << "Failed to load lidar extrinsic: " << kLidarExtrinsicsPath;
      return false;
    }
    AINFO << "Yunle PCL-OMP NDT lidar extrinsic loaded from "
          << kLidarExtrinsicsPath << ": translation=("
          << lidar_extrinsic_.translation().x() << ", "
          << lidar_extrinsic_.translation().y() << ", "
          << lidar_extrinsic_.translation().z() << ")";

    if (!LoadMap()) {
      return false;
    }

    ndt_.setTransformationEpsilon(kNdtTransformationEpsilon);
    ndt_.setStepSize(kNdtStepSize);
    ndt_.setResolution(kNdtResolution);
    ndt_.setMaximumIterations(kNdtMaxIterations);
    ndt_.setNumThreads(kNdtNumThreads);
    ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt_.setInputTarget(map_cloud_);

    odometry_reader_ = node_->CreateReader<localization::Gps>(
        kNdtOdometryTopic,
        [this](const std::shared_ptr<localization::Gps>& odometry) {
          OnOdometry(odometry);
        });
    point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
        kNdtPointCloudTopic,
        [this](const std::shared_ptr<drivers::PointCloud>& cloud) {
          OnPointCloud(cloud);
        });
    if (odometry_reader_ == nullptr || point_cloud_reader_ == nullptr) {
      AERROR << "Failed to create Yunle PCL-OMP NDT readers.";
      return false;
    }

    AINFO << "Yunle PCL-OMP NDT localizer started. map=" << kMapPcdPath
          << " offset=(" << kMapOffsetX << ", " << kMapOffsetY << ", "
          << kMapOffsetZ << ") input_cloud=" << kNdtPointCloudTopic
          << " input_odometry=" << kNdtOdometryTopic << " raw_outputs="
          << kRawLocalizationTopic << ", " << kRawNdtLidarTopic;
    return true;
  }

 private:
  bool LoadMap() {
    PclCloud::Ptr raw_map(new PclCloud());
    if (pcl::io::loadPCDFile<PclPoint>(kMapPcdPath, *raw_map) != 0 ||
        raw_map->empty()) {
      AERROR << "Failed to load Yunle indoor PCD map: " << kMapPcdPath;
      return false;
    }

    for (auto& point : raw_map->points) {
      point.x += static_cast<float>(kMapOffsetX);
      point.y += static_cast<float>(kMapOffsetY);
      point.z += static_cast<float>(kMapOffsetZ);
    }

    pcl::VoxelGrid<PclPoint> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(kMapVoxelLeafM, kMapVoxelLeafM,
                                  kMapVoxelLeafM);
    voxel_grid_filter.setInputCloud(raw_map);
    map_cloud_.reset(new PclCloud());
    voxel_grid_filter.filter(*map_cloud_);

    if (map_cloud_->empty()) {
      AERROR << "Yunle indoor PCD map became empty after voxel filtering.";
      return false;
    }

    AINFO << "Loaded Yunle indoor PCD map: raw_points=" << raw_map->size()
          << " filtered_points=" << map_cloud_->size()
          << " map_voxel_leaf=" << kMapVoxelLeafM;
    return true;
  }

  void OnOdometry(const std::shared_ptr<localization::Gps>& odometry) {
    if (odometry == nullptr || !odometry->has_header() ||
        !odometry->has_localization() ||
        !IsFinitePose(odometry->localization())) {
      AWARN_EVERY(50) << "Ignoring incomplete PCL-OMP NDT odometry prior.";
      return;
    }
    const double timestamp_sec = odometry->header().timestamp_sec();
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0) {
      AWARN_EVERY(50) << "Ignoring invalid PCL-OMP NDT odometry time.";
      return;
    }

    std::lock_guard<std::mutex> lock(odometry_mutex_);
    odometry_cache_.push_back(odometry);
    while (odometry_cache_.size() > kMaxOdometryCacheSize) {
      odometry_cache_.pop_front();
    }
  }

  bool FindNearestOdometry(
      double timestamp_sec,
      std::shared_ptr<localization::Gps>* nearest_odometry) const {
    if (nearest_odometry == nullptr) {
      return false;
    }
    double nearest_difference = std::numeric_limits<double>::max();
    std::lock_guard<std::mutex> lock(odometry_mutex_);
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

  void OnPointCloud(const std::shared_ptr<drivers::PointCloud>& cloud) {
    if (cloud == nullptr || cloud->point_size() == 0) {
      return;
    }
    const double timestamp_sec = PointCloudTimeSec(*cloud);
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0) {
      AWARN_EVERY(50) << "Ignoring PCL-OMP NDT point cloud with invalid time.";
      return;
    }

    std::shared_ptr<localization::Gps> odometry;
    if (!FindNearestOdometry(timestamp_sec, &odometry)) {
      AWARN_EVERY(20) << "No nearby odometry prior for PCL-OMP NDT cloud time "
                      << timestamp_sec;
      return;
    }

    PclCloud::Ptr filtered_scan = ConvertAndFilterCloud(*cloud);
    if (filtered_scan->size() < kMinFilteredScanPoints) {
      AWARN_EVERY(20) << "PCL-OMP NDT scan has too few filtered points: "
                      << filtered_scan->size();
      return;
    }

    const Eigen::Affine3d odometry_pose =
        PoseToAffine(odometry->localization());
    const Eigen::Matrix4f initial_guess =
        odometry_pose.matrix().cast<float>();

    PclCloud::Ptr aligned_scan(new PclCloud());
    Eigen::Matrix4f final_transform = Eigen::Matrix4f::Identity();
    double transform_probability = 0.0;
    int iteration_num = 0;
    bool has_converged = false;
    {
      std::lock_guard<std::mutex> lock(ndt_mutex_);
      ndt_.setInputSource(filtered_scan);
      ndt_.align(*aligned_scan, initial_guess);
      final_transform = ndt_.getFinalTransformation();
      transform_probability = ndt_.getTransformationProbability();
      iteration_num = ndt_.getFinalNumIteration();
      has_converged = ndt_.hasConverged();
    }

    const bool accepted =
        has_converged && iteration_num < kNdtMaxIterations + 2 &&
        transform_probability >= kMinTransformProbability;
    AINFO_EVERY(20) << "PCL-OMP NDT frame summary: raw_points="
                    << cloud->point_size()
                    << " filtered_points=" << filtered_scan->size()
                    << " converged=" << has_converged
                    << " iteration=" << iteration_num
                    << " trans_prob=" << transform_probability
                    << " accepted=" << accepted;
    if (!accepted) {
      AWARN_EVERY(10) << "Skipping PCL-OMP NDT raw localization because match "
                      << "did not pass convergence gates.";
      return;
    }

    Eigen::Affine3d result_pose = Eigen::Affine3d::Identity();
    result_pose.matrix() = final_transform.cast<double>();
    auto raw_localization =
        ComposeLocalization(timestamp_sec, result_pose, *odometry, true);
    auto raw_lidar =
        ComposeLocalization(timestamp_sec, result_pose, *odometry, false);
    raw_localization_writer_->Write(raw_localization);
    raw_ndt_lidar_writer_->Write(raw_lidar);
  }

  PclCloud::Ptr ConvertAndFilterCloud(const drivers::PointCloud& cloud) const {
    PclCloud::Ptr raw_scan(new PclCloud());
    raw_scan->reserve(static_cast<std::size_t>(cloud.point_size()));
    const double max_range_sq = kMaxScanRangeM * kMaxScanRangeM;
    for (const auto& point : cloud.point()) {
      const float x = point.x();
      const float y = point.y();
      const float z = point.z();
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double range_sq = static_cast<double>(x) * x +
                              static_cast<double>(y) * y;
      if (range_sq > max_range_sq) {
        continue;
      }
      PclPoint pcl_point;
      pcl_point.x = x;
      pcl_point.y = y;
      pcl_point.z = z;
      pcl_point.intensity = static_cast<float>(point.intensity());
      raw_scan->push_back(pcl_point);
    }
    raw_scan->width = static_cast<uint32_t>(raw_scan->size());
    raw_scan->height = 1;
    raw_scan->is_dense = false;

    PclCloud::Ptr base_scan(new PclCloud());
    pcl::transformPointCloud(*raw_scan, *base_scan,
                             lidar_extrinsic_.matrix().cast<float>());

    PclCloud::Ptr voxel_scan(new PclCloud());
    pcl::VoxelGrid<PclPoint> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(kScanVoxelLeafM, kScanVoxelLeafM,
                                  kScanVoxelLeafZ);
    voxel_grid_filter.setInputCloud(base_scan);
    voxel_grid_filter.filter(*voxel_scan);

    PclCloud::Ptr filtered_scan(new PclCloud());
    pcl::PassThrough<PclPoint> pass_through_filter;
    pass_through_filter.setInputCloud(voxel_scan);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(kMinScanZ, kMaxScanZ);
    pass_through_filter.filter(*filtered_scan);
    return filtered_scan;
  }

  std::shared_ptr<localization::LocalizationEstimate> ComposeLocalization(
      double measurement_time, const Eigen::Affine3d& pose,
      const localization::Gps& odometry, bool copy_motion_fields) {
    auto localization =
        std::make_shared<localization::LocalizationEstimate>();
    auto* header = localization->mutable_header();
    header->set_module_name(node_->Name());
    header->set_timestamp_sec(cyber::Clock::NowInSeconds());
    header->set_sequence_num(++localization_sequence_num_);
    localization->set_measurement_time(measurement_time);

    auto* mutable_pose = localization->mutable_pose();
    FillPose(pose, mutable_pose);
    if (copy_motion_fields && odometry.has_localization()) {
      CopyMotionFields(odometry.localization(), mutable_pose);
    }
    return localization;
  }

  PclCloud::Ptr map_cloud_;
  Eigen::Affine3d lidar_extrinsic_ = Eigen::Affine3d::Identity();
  pclomp::NormalDistributionsTransform<PclPoint, PclPoint> ndt_;

  mutable std::mutex odometry_mutex_;
  std::deque<std::shared_ptr<localization::Gps>> odometry_cache_;
  std::mutex ndt_mutex_;

  uint32_t localization_sequence_num_ = 0;

  std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
  std::shared_ptr<cyber::Reader<localization::Gps>> odometry_reader_;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      raw_localization_writer_;
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      raw_ndt_lidar_writer_;
};

CYBER_REGISTER_COMPONENT(YunleIndoorPclOmpNdtLocalizer)

}  // namespace yunle
}  // namespace apollo
