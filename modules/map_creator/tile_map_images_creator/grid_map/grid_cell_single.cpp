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

#include "grid_cell_single.h"
#include <cstdlib>

namespace apollo {
namespace tile_map_images_creator {

GridCellSingle::GridCellSingle() {
    intensity_ = 0.0;
    intensity_var_ = 0.0;
    altitude_ = 0.0;
    altitude_var_ = 0.0;
    altitude_relative_ = 0.0;
    altitude_relative_min_ = 0.0;
    altitude_relative_max_ = 0.0;
    r_ = 0.0;
    r_var_ = 0.0;
    g_ = 0.0;
    g_var_ = 0.0;
    b_ = 0.0;
    b_var_ = 0.0;
    count_ = 0;
    layer_ = 0;
    index_x_ = 0;
    index_y_ = 0;
}

GridCellSingle::GridCellSingle(const GridCellSingle& map_cell) {
    intensity_ = map_cell.intensity_;
    intensity_var_ = map_cell.intensity_var_;
    altitude_ = map_cell.altitude_;
    altitude_var_ = map_cell.altitude_var_;
    altitude_relative_ = map_cell.altitude_relative_;
    altitude_relative_min_ = map_cell.altitude_relative_min_;
    altitude_relative_max_ = map_cell.altitude_relative_max_;
    r_ = map_cell.r_;
    r_var_ = map_cell.r_var_;
    g_ = map_cell.g_;
    g_var_ = map_cell.g_var_;
    b_ = map_cell.b_;
    b_var_ = map_cell.b_var_;
    count_ = map_cell.count_;
    layer_ = map_cell.layer_;
    index_x_ = map_cell.index_x_;
    index_y_ = map_cell.index_y_;
    resolution_ = map_cell.resolution_;
}

void GridCellSingle::CalAvgVar(double& _average, double& _variance, const double new_val) {
    double v1 = new_val - _average;
    double value = v1 / count_;
    _average += value;
    double v2 = new_val - _average;
    _variance = (static_cast<double>(count_ - 1) * _variance + v1 * v2) / count_;
}

void GridCellSingle::AddSample(
        double altitude,
        double altitude_relative,
        double altitude_relative_var,
        double altitude_relative_min,
        double altitude_relative_max,
        unsigned int count,
        double intensity,
        unsigned char r,
        unsigned char g,
        unsigned char b) {
    MergeMember(r_, r_var_, count_, r, 0, count);
    MergeMember(g_, g_var_, count_, g, 0, count);
    MergeMember(b_, b_var_, count_, b, 0, count);
    MergeMember(altitude_, altitude_var_, count_, altitude, 0, count);
    MergeMember(altitude_relative_, altitude_relative_var_, count_, altitude_relative, altitude_relative_var, count);
    MergeMember(intensity_, intensity_var_, count_, intensity, 0, count);
    count_ += count;
}

void GridCellSingle::MergeMember(
        double& value0,
        double& value0_var,
        unsigned int count0,
        const double value1,
        const double value1_var,
        unsigned int count1) {
    unsigned int new_count = count0 + count1;
    double p0 = static_cast<double>(count0) / new_count;
    double p1 = static_cast<double>(count1) / new_count;
    double value_diff = value0 - value1;
    value0 = value0 * p0 + value1 * p1;
    value0_var = value0_var * p0 + value1_var * p1 + value_diff * value_diff * p0 * p1;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
