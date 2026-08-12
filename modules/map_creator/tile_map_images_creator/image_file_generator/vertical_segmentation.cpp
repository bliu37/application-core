/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "vertical_segmentation.h"

namespace apollo {
namespace tile_map_images_creator {

bool VerticalSegmentation::Segmentation(const apollo::tile_map_images_creator::GridCellSingle &cell) const {
    if (!conf_.has_enable_height_relative_converter() || !conf_.enable_height_relative_converter()) {
        return false;
    }

    if (conf_.has_height_count_threshold() && cell.count_ <= static_cast<size_t>(conf_.height_count_threshold())) {
        return false;
    }

    if (conf_.has_height_avg_threshold() && cell.altitude_relative_ <= conf_.height_avg_threshold()) {
        return false;
    }

    if (conf_.has_height_var_threshold() && cell.altitude_relative_var_ <= conf_.height_var_threshold()) {
        return false;
    }

    return true;
}

}
}
