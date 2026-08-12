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
#pragma once

#include <memory>
#include <vector>
#include <string>
#include "cyber/cyber.h"
#include "modules/map_creator/tile_map_images_creator/time_align/record_align_adapter.h"
#include "modules/map_creator/tile_map_images_creator/time_align/vector_align_adapter.hpp"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/map_creator/tile_map_images_creator/proto/slam_pose_result.pb.h"
#include "frame_loader.h"

namespace apollo {
namespace tile_map_images_creator {
class SingleSourceFrameLoader final : public FrameLoaderInterface {
public:
    SingleSourceFrameLoader() = default;

    SingleSourceFrameLoader(
            const std::vector<std::string>& record_files,
            const std::string& localization_channel_name,
            const std::string& pcd_channel_name);

    bool Init() override;

    bool LoadNextFrame(ImageCreatorFrame& frame) override;

private:
    std::string pcd_channel_name_, localization_channel_name_;
    std::vector<std::string> record_files_;
    std::shared_ptr<RecordAlignAdapter<apollo::drivers::PointCloud, apollo::localization::LocalizationEstimate>>
            record_source_adapter_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
