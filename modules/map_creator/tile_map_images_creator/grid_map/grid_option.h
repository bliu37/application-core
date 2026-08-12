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

#include <string>

namespace apollo {
namespace tile_map_images_creator {

const double MIN_COORDINATE_X = 0.0;
const double MIN_COORDINATE_Y = 4000000.0;
const double MIN_COORDINATE_Z = 0.0;
const double MAX_COORDINATE_X = 1000448.0;
const double MAX_COORDINATE_Y = 10000384.0;

struct GridOption {
    GridOption() = default;
    /**@brief copy constructer. */
    GridOption(const GridOption* option);
   /**@brief compare with another config. */
    bool Compare(const GridOption* other_option);
    /**@brief set map folder. */
    void set_map_folder(const std::string& map_folder) {
        map_folder_path = map_folder;
    }
    /**@brief set resolution */
    void set_resolution(const double in_resolution) {
        resolution = in_resolution;
    }
    void set_resolution_id(const int in_resolution_id) {
        resolution_id = in_resolution_id;
    }
    /**@brief set zone id */
    void set_zone_id(unsigned int in_zone_id) {
        zone_id = in_zone_id;
    }
    /**@brief set tile_size */
    void set_matrix_size(const unsigned int in_matrix_size) {
        matrix_size = in_matrix_size;
    }
    void set_min_x(const double x) {
        min_x = x;
    }
    void set_min_y(const double y) {
        min_y = y;
    }
    void set_min_z(const double z) {
        min_z = z;
    }
    double map_version;
    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double resolution;
    double matrix_resolution;
    int resolution_id;
    unsigned int matrix_size;
    unsigned int zone_id;
    std::string map_folder_path;
};

}
}
