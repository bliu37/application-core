/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <chrono>

#include "modules/drivers/lidar/lslidar/component/lslidar_component.h"

// Apollo 11 LS-C16 V4 component override: keep polling when the scan queue is
// temporarily empty instead of permanently ending the conversion thread.

namespace apollo {
namespace drivers {
namespace lslidar {

LslidarComponent::~LslidarComponent() {
  if (device_thread_->joinable()) {
    device_thread_->join();
  }
  if (pointcloud_convert_thread_->joinable()) {
    pointcloud_convert_thread_->join();
  }
}

bool LslidarComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "load config error, file:" << config_file_path_;
    return false;
  }

  this->InitBase(conf_.config_base());

  driver::LslidarDriver* driver =
      driver::LslidarDriverFactory::CreateDriver(conf_);
  if (!driver) {
    return false;
  }
  dvr_.reset(driver);
  dvr_->Init();

  device_thread_ = std::make_shared<std::thread>(
      std::bind(&LslidarComponent::DevicePollProcess, this));
  pointcloud_convert_thread_ = std::make_shared<std::thread>(
      &LslidarComponent::ScanQueuePollProcess, this);

  converter_ = std::make_shared<parser::Convert>();
  converter_->init(conf_);

  return true;
}

void LslidarComponent::ReadScanCallback(
    const std::shared_ptr<LslidarScan>& scan_message) {
  HandleScanFrame(scan_message);
}

void LslidarComponent::DevicePollProcess() {
  while (!apollo::cyber::IsShutdown()) {
    auto scan = std::make_shared<LslidarScan>();

    if (dvr_->Poll(scan)) {
      common::util::FillHeader("lslidar", scan.get());
      this->WriteScan(scan);
      scan_queue_.push(scan);
    } else {
      AWARN << "device poll failed";
    }
  }

  AERROR << "CompLslidarDriver thread exit";
}

void LslidarComponent::ScanQueuePollProcess() {
  std::shared_ptr<LslidarScan> scan_frame;
  while (!apollo::cyber::IsShutdown()) {
    if (scan_queue_.popWait(scan_frame)) {
      HandleScanFrame(scan_frame);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void LslidarComponent::HandleScanFrame(
    const std::shared_ptr<LslidarScan>& scan_frame) {
  auto point_cloud_out = this->AllocatePointCloud();
  converter_->ConvertPacketsToPointcloud(scan_frame, point_cloud_out);
  this->WritePointCloud(point_cloud_out);
}

}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
