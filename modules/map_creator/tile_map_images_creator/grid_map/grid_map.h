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

#include <map>

#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"

#include "modules/map_creator/tile_map_images_creator/image_file_generator/image_file_generator.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/LRU_cache.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_matrix.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_option.h"

#pragma once

namespace apollo {
namespace tile_map_images_creator {

class GridMap {
public:
    /**@brief The default constructor. */
    GridMap() = default;
    /**@brief The constructor with option. */
    GridMap(std::shared_ptr<GridOption> option, ImagesCreatorConf conf);
    /**@brief The default destructor. */
    ~GridMap();
    /**@brief Get the matrixs in this map. */
    inline std::map<MatrixIndex, std::shared_ptr<GridMatrix>>& get_matrixs() {
        return matrixs_;
    }
    /**@brief Set point to multi layer matrix. */
    bool SetValue(
            double x,
            double y,
            double z,
            double z_relative,
            double z_relative_min,
            double z_relative_max,
            unsigned int count,
            double intensity,
            unsigned char r,
            unsigned char g,
            unsigned char b,
            int layer = 0);

    bool SaveMatrixToDisk(MatrixIndex matrix_index);

    bool LoadValueFromDisk(MatrixIndex matrix_index);

    /**@brief Get the resolution of this map. */
    inline float get_resolution() {
        return resolution_;
    }

    /**@brief Get the id of this map. */
    int get_resolution_id() {
        return resolution_id_;
    }

private:
    double resolution_;
    std::shared_ptr<GridOption> option_;
    std::map<MatrixIndex, std::shared_ptr<GridMatrix>> matrixs_;
    LRUcache<MatrixIndex> lru_cache_;
    InputOutputConf io_conf_;
    int resolution_id_;
    std::shared_ptr<ImageFileGenerator> image_file_generator_;

    // 优化内存
    // std::string _bin_output_dir, _image_output_dir;
    // bool _use_LRU_cache;
    // int _LRU_cache_size;
    // cybertron::map_maker::LRUcache<Matrix_index> _lru_cache;
    long long total_query_num_ = 0, total_miss_num_ = 0;
};

}  // namespace tile_map_images_creator
}  // namespace apollo