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
 *****************************************************************************/

#include "modules/map_creator/tile_map_images_creator/images_creator/images_frame_producer.h"

namespace apollo {
namespace tile_map_images_creator {

bool ImageFrameProducer::Init(const std::vector<std::string>& record_files, std::string* err_message) {
    if (conf_.has_slam_mode_selection_conf() && conf_.slam_mode_selection_conf().enable_slam_mode()) {
        const auto& slam_mode_conf = conf_.slam_mode_selection_conf();
        frame_loader_ = std::make_unique<MultiSourceFrameLoader>(
                record_files, slam_mode_conf.slam_pose_path(), conf_.reader_conf().point_cloud_channel());
        if (!frame_loader_->Init()) {
            WRITE_ERR_MESSAGE("frame loader init failed in slam mode");
            return false;
        }

        AINFO << boost::format(
                         "Slam mode selected. Frame loader init with multi-source loader. Point cloud load from "
                         "channel channel = %s, "
                         "localization load from binary file path = %s")
                        % conf_.reader_conf().point_cloud_channel() % slam_mode_conf.slam_pose_path();
    } else {
        // RTK模式
        frame_loader_ = std::make_unique<SingleSourceFrameLoader>(
                record_files, conf_.reader_conf().localization_channel(), conf_.reader_conf().point_cloud_channel());

        if (!frame_loader_->Init()) {
            WRITE_ERR_MESSAGE("frame loader init failed in rtk mode");
            return false;
        }

        AINFO << boost::format(
                         "RTK mode selected. Frame loader init with single-source loader. Point cloud load from "
                         "channel channel = %s, "
                         "localization channel = %s")
                        % conf_.reader_conf().point_cloud_channel() % conf_.reader_conf().localization_channel();
    }

    return true;
}

bool ImageFrameProducer::LoadNextFrame(ImageCreatorFrame& frame) {
    while (frame_loader_->LoadNextFrame(frame)) {
        std::shared_ptr<common::PointENU> now_position
                = std::make_shared<common::PointENU>(frame.localization_frame_->pose().position());
        if (last_inserted_position_ != nullptr) {
            double distance_to_last_position = hypot(
                    last_inserted_position_->x() - now_position->x(), last_inserted_position_->y() - now_position->y());
            if (distance_to_last_position < conf_.sample_distance()) {
                AINFO << boost::format(
                                 "distance to last position = %.6f, less than sample distance %.6f, point cloud "
                                 "skip")
                                % distance_to_last_position % conf_.sample_distance();
                continue;
            }
        }

        double pcd_timestamp = frame.pcd_frame_->measurement_time();
        double localization_timestamp = frame.localization_frame_->measurement_time();
        if (fabs(pcd_timestamp - localization_timestamp) > conf_.reader_conf().maximum_timestamp_interval()) {
            AINFO << boost::format(
                             "timestamp interval between pcd and localization is too large, interval = %.6f > %.6f")
                            % fabs(pcd_timestamp - localization_timestamp)
                            % conf_.reader_conf().maximum_timestamp_interval();
            continue;
        }

        last_inserted_position_ = now_position;
        // todo 下一工序
        return true;
    }
    return false;
}

}  // namespace tile_map_images_creator
}  // namespace apollo