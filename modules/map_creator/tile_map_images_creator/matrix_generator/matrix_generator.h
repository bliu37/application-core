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

// #include "grey_scale_image_generator.h"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"

#include "modules/map_creator/tile_map_images_creator/common/frame.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_map.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_matrix.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_option.h"
// #include "modules/map_creator/tile_map_images_creator/matrix_generator/intensity_convertor.h"

/**
 * @namespace apollo::tile_map_images_creator
 * @brief apollo::tile_map_images_creator
 */
namespace apollo {
namespace tile_map_images_creator {

class MatrixGenerator {
public:
    MatrixGenerator() = default;
    void Init(ImagesCreatorConf conf);

    void AddMapOption(double resolution, int resolution_id = 0);
    void AddMapOption(std::shared_ptr<GridOption> option);

    bool InsertPointCloud(const std::vector<PointXYZIT_D> &points, const common::PointENU& now_localization);

    bool SaveToDisk();

private:
    std::vector<std::shared_ptr<GridMap>> maps_;
    ImagesCreatorConf conf_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
