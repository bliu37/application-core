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
#pragma once

#include "boost/format.hpp"
#include "intensity_convertor.h"
#include "vertical_segmentation.h"
#include "opencv2/opencv.hpp"
#include "string"

#include "cyber/cyber.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_matrix.h"
namespace apollo {
namespace tile_map_images_creator {

class ImageFileGenerator {
public:
    ImageFileGenerator() = default;

    void SetPixelIntensityQuantizer(const std::shared_ptr<IntensityConverter> quantizer) {
        pixel_quantizer_ = quantizer;
    }

    void SetVerticalSegmentation(const std::shared_ptr<VerticalSegmentation> vertical_segmentation) {
        vertical_segmentation_ = vertical_segmentation;
    }

    void SetImageDirPath(const std::string& image_dir) {
        image_dir_path_ = image_dir;
    }

    bool SaveImageFile(const GridMatrix& matrix);

    bool SaveImageFile(const GridMatrix& matrix, std::string image_path);

    cv::Mat GetImageMatrix(const GridMatrix& matrix) const;

    std::string image_dir_path_;
    std::shared_ptr<IntensityConverter> pixel_quantizer_;
    std::shared_ptr<VerticalSegmentation> vertical_segmentation_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
