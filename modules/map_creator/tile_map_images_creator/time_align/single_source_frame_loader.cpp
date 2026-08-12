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

#include "modules/map_creator/tile_map_images_creator/time_align/single_source_frame_loader.h"

#include "modules/map_creator/tile_map_images_creator/common/utils.h"

namespace apollo {
namespace tile_map_images_creator {

SingleSourceFrameLoader::SingleSourceFrameLoader(
        const std::vector<std::string>& record_files,
        const std::string& localization_channel_name,
        const std::string& pcd_channel_name) {
    record_files_.assign(record_files.begin(), record_files.end());
    localization_channel_name_ = localization_channel_name;
    pcd_channel_name_ = pcd_channel_name;
}

bool SingleSourceFrameLoader::Init() {
    record_source_adapter_ = std::make_shared<
            RecordAlignAdapter<apollo::drivers::PointCloud, apollo::localization::LocalizationEstimate>>(record_files_);

    if (!record_source_adapter_) {
        AERROR << "source adapter create fail, record_source_adapter_ = " << record_source_adapter_.get();
        return false;
    }

    record_source_adapter_->SetChannels(pcd_channel_name_, localization_channel_name_);
    record_source_adapter_->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::drivers::PointCloud>& x) -> uint64_t {
                return GetNanosecondTimestampFromSecondTimestamp(x->measurement_time());
            },
            [](const std::shared_ptr<apollo::localization::LocalizationEstimate>& x) -> u_int64_t {
                return GetNanosecondTimestampFromSecondTimestamp(x->measurement_time());
            });
    AINFO << "pcd and localization source adapter (record) init success. ";
    return true;
}

bool SingleSourceFrameLoader::LoadNextFrame(ImageCreatorFrame& frame) {
    std::tuple<
            std::shared_ptr<apollo::drivers::PointCloud>,
            std::shared_ptr<apollo::localization::LocalizationEstimate>>
            tuple_frame;
    uint64_t primary_timestamp = 0;

    if (!record_source_adapter_->GetNextFrame(tuple_frame, primary_timestamp)) {
        AINFO << "record source adapter load unsuccessfully, maybe records reach to the end. timestamp = "
              << primary_timestamp;
        return false;
    }
    frame.pcd_frame_ = std::get<0>(tuple_frame);
    frame.localization_frame_ = std::get<1>(tuple_frame);
    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
