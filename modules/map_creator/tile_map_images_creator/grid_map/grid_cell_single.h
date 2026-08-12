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

#include <iostream>

#pragma once

namespace apollo {
namespace tile_map_images_creator {

struct GridCellSingle {
    /**@brief The default constructor. */
    GridCellSingle();
    /**@brief The copy constructor. */
    GridCellSingle(const GridCellSingle& map_cell);
    /**@brief calculate average and variance when add sample. */
    void CalAvgVar(double& _average, double& _variance, const double new_val);
    /**@brief Add an sample to the single map cell. */
    void AddSample(
            double altitude,
            double altitude_relative,
            double altitude_relative_var,
            double altitude_relative_min,
            double altitude_relative_max,
            unsigned int count,
            double intensity,
            unsigned char r,
            unsigned char g,
            unsigned char b);
    /**@brief merge member when merge two cells. */
    void MergeMember(
            double& value0,
            double& value0_var,
            unsigned int count0,
            const double value1,
            const double value1_var,
            unsigned int count1);
    /**@brief Set index x and index y. */
    inline void set_index(unsigned int index_x, unsigned int index_y) {
        index_x_ = index_x;
        index_y_ = index_y;
    }
    /**@brief Set index x and index y. */
    inline void set_resolution(float resolution) {
        resolution_ = resolution;
    }
    /**@brief Set layer. */
    inline void set_layer(unsigned char layer) {
        layer_ = layer;
    }

    /**@brief The average intensity value. */
    double intensity_;
    /**@brief The variance intensity value. */
    double intensity_var_;
    /**@brief The average altitude value. */
    double altitude_;
    /**@brief The variance altitude value. */
    double altitude_var_;
    double altitude_relative_;
    double altitude_relative_var_;
    double altitude_relative_min_;
    double altitude_relative_max_;

    /**@brief The average r value. */
    double r_;
    /**@brief The variance r value. */
    double r_var_;
    /**@brief The average g value. */
    double g_;
    /**@brief The variance g value. */
    double g_var_;
    /**@brief The average b value. */
    double b_;
    /**@brief The variance b value. */
    double b_var_;
    /**@brief The index x of this cell. */
    unsigned int index_x_;
    /**@brief The index y of this cell. */
    unsigned int index_y_;
    /**@brief The resolution of this cell. */
    float resolution_;
    /**@brief The count of point in this cell. */
    unsigned int count_;
    /**@brief The layer of this cell. */
    unsigned char layer_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
