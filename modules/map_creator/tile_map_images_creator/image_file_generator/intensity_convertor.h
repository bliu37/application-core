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
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_cell_single.h"

namespace apollo {
namespace tile_map_images_creator {

class IntensityConverter {
public:
    explicit IntensityConverter() = default;
    IntensityConverter(apollo::tile_map_images_creator::IntensityConverterConf conf);

    unsigned char Convert(const GridCellSingle& cell) const;

    unsigned char SigmoidConverter(double x, double sigmoid_x0, double sigmoid_k, unsigned upper_limit = 255) const;

private:
    double sigmoid_k_, sigmoid_x0_;
    bool enable_sigmoid_converter_ = false;

    apollo::tile_map_images_creator::IntensityConverterConf conf_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo