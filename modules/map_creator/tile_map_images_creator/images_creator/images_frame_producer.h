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

#include <boost/format.hpp>

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"

#include "modules/map_creator/tile_map_images_creator/common/macro.h"
#include "modules/map_creator/tile_map_images_creator/time_align/frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/multi_source_frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/single_source_frame_loader.h"

namespace apollo {
namespace tile_map_images_creator {

class ImageFrameProducer {
public:
    ImageFrameProducer(const ImagesCreatorConf& conf) : conf_(conf) {}

    bool LoadNextFrame(ImageCreatorFrame& frame);

    bool Init(const std::vector<std::string>& record_files, std::string* err_message);

private:
    std::unique_ptr<FrameLoaderInterface> frame_loader_ = nullptr;
    std::shared_ptr<apollo::common::PointENU> last_inserted_position_ = nullptr;

    ImagesCreatorConf conf_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
