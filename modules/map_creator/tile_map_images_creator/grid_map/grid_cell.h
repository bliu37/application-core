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

#include <vector>

#include "grid_cell_single.h"
#define IDL_CAR_NUM_RESERVED_MAP_LAYER 1

namespace apollo {
namespace tile_map_images_creator {

/**@brief The multiple layers of the cell. */
struct GridCell {
    /**@brief The default constructor. */
    GridCell() = default;
    /**@brief Reset to default value. */
    void Reset();
    /**@brief Set the value of a layer. The target layer is found according to the altitude. */
    void SetValue(
            double altitude,
            double altitude_relative,
            double altitude_relative_var,
            double altitude_relative_min,
            double altitude_relative_max,
            unsigned int count,
            double intensity,
            unsigned char r,
            unsigned char g,
            unsigned char b,
            int layer = 0);
    /**@brief Get CellSingles from . */
    void GetSingleCells(std::vector<GridCellSingle>& single_cells);

    GridCellSingle map_cells_[IDL_CAR_NUM_RESERVED_MAP_LAYER];
    unsigned char map_cell_size_;
    unsigned int index_x_;
    unsigned int index_y_;
    float resolution_;
    inline void set_index_x(unsigned int index_x) {
        index_x_ = index_x;
    }
    inline void set_index_y(unsigned int index_y) {
        index_y_ = index_y;
    }
    inline void set_resolution(float resolution) {
        resolution_ = resolution;
    }
};

}  // namespace tile_map_images_creator
}  // namespace apollo
