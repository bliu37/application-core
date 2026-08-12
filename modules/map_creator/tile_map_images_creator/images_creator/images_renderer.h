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

#include <atomic>
#include <memory>

#include "boost/format.hpp"

#include "cyber/cyber.h"
#include "modules/map_creator/tile_map_images_creator/common/utils.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_creator_result.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_renderer.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/pcd_creator.h"
#include "modules/map_creator/tile_map_images_creator/matrix_generator/matrix_generator.h"
#include "modules/map_creator/tile_map_images_creator/point_cloud_filter/point_cloud_preprocessing_filter.h"
#include "modules/map_creator/tile_map_images_creator/tiles_creator/tiles_creator.h"

namespace apollo {
namespace tile_map_images_creator {

class ImageRenderer {
public:
    ImageRenderer(const ImagesCreatorConf& conf) : conf_(conf) {}

    bool Init(uint64_t total_message);

    ImagesCreatorResult AppendResultFrame(const ResultFrame& result_frame);

    ImagesCreatorResult FinalOperation();

    void ForwardProgress() {
        ++processed_message_num_;
        progress_ = 99.0 * processed_message_num_ / total_message_num_;
    }

    float GetProgress() const {  // async
        return progress_;
    };

    std::shared_ptr<TilesCreator> tiles_creator_ = nullptr;
    std::shared_ptr<PointCloudPreprocessingFilter> point_cloud_filter_;
    std::shared_ptr<MatrixGenerator> matrix_generator_;

    std::shared_ptr<PcdCreator> pcd_debug_file_creator;
    bool enable_pcd_output_ = false;

    uint64_t total_message_num_ = 0;
    std::atomic_uint64_t processed_message_num_{0};
    float progress_ = 0.0;

    ImagesCreatorConf conf_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
