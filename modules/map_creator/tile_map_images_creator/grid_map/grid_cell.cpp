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

#include "grid_cell.h"

namespace apollo {
namespace tile_map_images_creator {

void GridCell::GetSingleCells(std::vector<GridCellSingle>& single_cells) {
    for (auto& single_cell : map_cells_) {
        if (single_cell.count_ == 0) {
            continue;
        }
        single_cells.push_back(single_cell);
    }
}

void GridCell::SetValue(
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
        int layer) {
    GridCellSingle& single_cell = map_cells_[layer];
    map_cell_size_ = std::max(map_cell_size_, (unsigned char)layer);
    single_cell.set_index(index_x_, index_y_);
    single_cell.set_resolution(resolution_);
    single_cell.set_layer(layer);
    single_cell.AddSample(altitude, altitude_relative, altitude_relative_var, altitude_relative_min, altitude_relative_max, count, intensity, r, g, b);
}

}  // namespace tile_map_images_creator
}  // namespace apollo