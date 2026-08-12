/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 *****************************************************************************/ \
#include "modules/map_creator/tile_map_images_creator/time_align/multi_source_frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/common/utils.h"

namespace apollo {
namespace tile_map_images_creator {

MultiSourceFrameLoader::MultiSourceFrameLoader(
        const std::vector<std::string>& record_files,
        const std::string& localization_pose_bin_path,
        const std::string& pcd_channel_name) {
    record_files_.assign(record_files.begin(), record_files.end());
    localization_pose_bin_path_ = localization_pose_bin_path;
    pcd_channel_name_ = pcd_channel_name;
}

bool MultiSourceFrameLoader::Init() {
    std::unique_ptr<apollo::tile_map_images_creator::SlamPoseResult> slam_result
            = std::make_unique<apollo::tile_map_images_creator::SlamPoseResult>();
    if (!apollo::cyber::common::GetProtoFromBinaryFile(localization_pose_bin_path_, slam_result.get())) {
        AERROR << "load localization binary file fail, path = " << localization_pose_bin_path_;
        return false;
    }

    AINFO << slam_result->DebugString();
    if (!slam_result->slam_pose_result_size()) {
        AERROR << "localization binary file doesn't have valid pose, path = " << localization_pose_bin_path_;
        return false;
    }

    std::vector<std::shared_ptr<apollo::localization::LocalizationEstimate>> localization_vector;
    for (int i = 0; i < slam_result->slam_pose_result_size(); ++i) {
        std::shared_ptr<apollo::localization::LocalizationEstimate> localization_element
                = std::make_shared<apollo::localization::LocalizationEstimate>();

        localization_element->mutable_header()->CopyFrom(slam_result->slam_pose_result(i).header());
        localization_element->mutable_pose()->mutable_position()->set_x(
                slam_result->slam_pose_result(i).transform().translation().x());
        localization_element->mutable_pose()->mutable_position()->set_y(
                slam_result->slam_pose_result(i).transform().translation().y());
        localization_element->mutable_pose()->mutable_position()->set_z(
                slam_result->slam_pose_result(i).transform().translation().z());
        localization_element->mutable_pose()->mutable_orientation()->CopyFrom(
                slam_result->slam_pose_result(i).transform().rotation());
        localization_element->set_measurement_time(slam_result->slam_pose_result(i).header().timestamp_sec());

        localization_vector.emplace_back(localization_element);
    }

    pointcloud_source_adapter_ = std::make_shared<RecordAlignAdapter<apollo::drivers::PointCloud>>(record_files_);
    localization_source_adapter_ = std::make_shared<VectorAlignAdapter<apollo::localization::LocalizationEstimate>>();

    if (!localization_source_adapter_ || !pointcloud_source_adapter_) {
        AERROR << "source adapter create fail, localization_adapter = " << localization_source_adapter_.get()
               << " pointcloud_adapter = " << pointcloud_source_adapter_.get();
        return false;
    }

    pointcloud_source_adapter_->SetChannels(pcd_channel_name_);
    pointcloud_source_adapter_->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::drivers::PointCloud>& x) -> uint64_t {
                return GetNanosecondTimestampFromSecondTimestamp(x->measurement_time());
            });

    AINFO << "point cloud source adapter (record) init success. ";

    localization_source_adapter_->Init(localization_vector);
    localization_source_adapter_->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::localization::LocalizationEstimate>& x) -> u_int64_t {
                return GetNanosecondTimestampFromSecondTimestamp(x->measurement_time());
            });
    AINFO << "localization source adapter (vector) init success. ";
    return true;
}

bool MultiSourceFrameLoader::LoadNextFrame(ImageCreatorFrame& frame) {
    std::tuple<std::shared_ptr<apollo::drivers::PointCloud>> pcd_frame;
    uint64_t pcd_timestamp = 0;
    while (pointcloud_source_adapter_->GetNextFrame(pcd_frame, pcd_timestamp)) {
        frame.pcd_frame_ = std::get<0>(pcd_frame);
        AINFO << "Load next point cloud data success, timestamp = " << pcd_timestamp;

        std::tuple<std::shared_ptr<apollo::localization::LocalizationEstimate>> localization_frame;
        if (!localization_source_adapter_->GetNextFrameWithTimestamp(localization_frame, pcd_timestamp)) {
            AINFO << "Load localization data fail, discard point cloud frame with timestamp = " << pcd_timestamp;
            continue;
        }
        frame.localization_frame_ = std::get<0>(localization_frame);
        return true;
    }
    AINFO << "Load point cloud data fail, maybe finish";
    return false;
}

}  // namespace tile_map_images_creator
}  // namespace apollo