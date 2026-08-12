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

#include "modules/map_creator/tile_map_images_creator/images_creator/images_renderer.h"

namespace apollo {
namespace tile_map_images_creator {

bool ImageRenderer::Init(uint64_t total_message) {
    matrix_generator_ = std::make_shared<MatrixGenerator>();
    MatrixGeneratorConf matrix_generator_conf = conf_.matrix_generator_conf();
    (void)matrix_generator_->Init(conf_);
    matrix_generator_->AddMapOption(matrix_generator_conf.matrix_resolution(), matrix_generator_conf.matrix_id());

    point_cloud_filter_ = std::make_shared<PointCloudPreprocessingFilter>();
    (void)point_cloud_filter_->Init(conf_.point_cloud_processing_conf().filter_conf());

    pcd_debug_file_creator = std::make_shared<PcdCreator>();
    if (conf_.has_debug_conf() && conf_.debug_conf().enable_pcd_file_output()) {
        AINFO << "enable debug point cloud output";
        enable_pcd_output_ = true;
    }

    tiles_creator_ = std::make_shared<TilesCreator>(
            conf_.input_output_conf().images_output_dir(), conf_.matrix_generator_conf().matrix_id());

    total_message_num_ = total_message;
    return true;
}

ImagesCreatorResult ImageRenderer::AppendResultFrame(const ResultFrame& result_frame) {
    ImagesCreatorResult result;

    common::PointENU now_position;
    now_position.set_x(result_frame.localization_frame_->pose().position().x());
    now_position.set_y(result_frame.localization_frame_->pose().position().y());
    now_position.set_z(result_frame.localization_frame_->pose().position().z());

    point_cloud_filter_->UpdatePosition(std::make_shared<common::PointENU>(now_position));
    point_cloud_filter_->Filter(*result_frame.pcd_frame_);

    if (!matrix_generator_->InsertPointCloud(*result_frame.pcd_frame_, now_position)) {
        AERROR << "error occurred when inserting point cloud message";

        result.succeeded = false;
        result.message = "Point cloud insertion error";
        return result;
    }

    if (enable_pcd_output_) {
        pcd_debug_file_creator->ConcatPointCloud(*result_frame.pcd_frame_);
    }

    ForwardProgress();

    result.succeeded = true;
    result.output_dir = conf_.input_output_conf().images_output_dir();
    return result;
}

ImagesCreatorResult ImageRenderer::FinalOperation() {
    ImagesCreatorResult result;
    AINFO << "start image render final operation";

    if (!matrix_generator_->SaveToDisk()) {
        AERROR << boost::format("error occurred when storing matrixes to disk");
        result.succeeded = false;
        result.message = "Matrix storage error";
        return result;
    }

    if (enable_pcd_output_) {
        pcd_debug_file_creator->SaveCloudToPcd(conf_.debug_conf().debug_pcd_file_path());
        AINFO << "point cloud debug file saved, path = " << conf_.debug_conf().debug_pcd_file_path();
    }

    // generate multi level tile images
    auto future = tiles_creator_->Start();
    // Note: the generate tile images is very fast, do not calculate progress currently.
    future.wait();
    auto tiles_creator_result = future.get();
    if (!tiles_creator_result.succeeded) {
        AERROR << "generate multi level tile images failed: " << tiles_creator_result.message;
        result.succeeded = false;
        result.message = tiles_creator_result.message;
        return result;
    }
    AINFO << "generate multi level tile images succeeded";

    // write tiles.json file
    if (!DumpTilesJson(
                conf_.input_output_conf().images_output_dir(),
                &result.message,
                conf_.matrix_generator_conf().matrix_resolution(),
                conf_.matrix_generator_conf().matrix_id())) {
        AERROR << "write tiles.json failed: " << result.message;
        result.succeeded = false;
    }
    AINFO << "write tiles.json file succeeded";

    progress_ = 100.0;

    result.succeeded = true;
    result.output_dir = conf_.input_output_conf().images_output_dir();
    return result;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
