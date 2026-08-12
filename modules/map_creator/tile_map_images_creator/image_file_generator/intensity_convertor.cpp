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

#include "modules/map_creator/tile_map_images_creator/image_file_generator/intensity_convertor.h"

#include <cmath>

namespace apollo {
namespace tile_map_images_creator {

IntensityConverter::IntensityConverter(apollo::tile_map_images_creator::IntensityConverterConf conf) {
    if (conf.has_enable_sigmoid_converter() && conf.has_intensity_sigmoid_k() && conf.has_intensity_sigmoid_x0()) {
        sigmoid_k_ = conf.intensity_sigmoid_k();
        sigmoid_x0_ = conf.intensity_sigmoid_x0();
        enable_sigmoid_converter_ = conf.enable_sigmoid_converter();
    }

    conf_ = conf;
}

unsigned char IntensityConverter::Convert(const GridCellSingle& cell) const {
    // 对原有强度使用Sigmoid函数变换
    double intensity = cell.intensity_;

    unsigned char intensity_c = 0;
    if (enable_sigmoid_converter_) {
        intensity_c = SigmoidConverter(intensity, sigmoid_x0_, sigmoid_k_);
    }
    return intensity_c;
}

// 0-n
unsigned char IntensityConverter::SigmoidConverter(double x, double sigmoid_x0, double sigmoid_k, unsigned upper_limit)
        const {
    x = upper_limit * 1.0 / (1 + exp(-sigmoid_k * (x - sigmoid_x0)));
    unsigned char result_x = 0;
    if (x > upper_limit) {
        result_x = upper_limit;
    } else if (x < 0) {
        result_x = 0;
    } else {
        result_x = static_cast<unsigned char>(x);
    }
    return result_x;
}

}  // namespace tile_map_images_creator
}  // namespace apollo