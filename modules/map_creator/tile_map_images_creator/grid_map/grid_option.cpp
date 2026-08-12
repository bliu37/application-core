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

#include "grid_option.h"
#include <cmath>

namespace apollo {
namespace tile_map_images_creator {
bool GridOption::Compare(const GridOption* other_option) {
    const double compare_thre = 0.00001;
    if (fabs(map_version - other_option->map_version) > compare_thre || fabs(min_x - other_option->min_x) > compare_thre
        || fabs(min_y - other_option->min_y) > compare_thre || fabs(min_z - other_option->min_z) > compare_thre
        || fabs(max_x - other_option->max_x) > compare_thre || fabs(max_y - other_option->max_y) > compare_thre
        || fabs(resolution - other_option->resolution) > compare_thre || matrix_size != other_option->matrix_size
        || zone_id != other_option->zone_id) {
        return false;
    } else {
        return true;
    }
}

}
}