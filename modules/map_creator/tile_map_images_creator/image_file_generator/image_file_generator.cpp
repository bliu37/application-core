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
 *****************************************************************************/ \
#include "image_file_generator.h"

namespace apollo {
namespace tile_map_images_creator {

bool ImageFileGenerator::SaveImageFile(const GridMatrix& matrix) {
    MatrixIndex index = matrix.get_index();
    std::string relative_dir_path
            = (boost::format("/%u/%08d") % matrix.get_resolution_id() % index.index_y_).str();
    std::string absolute_dir_path = image_dir_path_ + relative_dir_path;
    if (!cyber::common::EnsureDirectory(absolute_dir_path)) {
        AERROR << "create image path failed! path = " << absolute_dir_path;
        return false;
    }
    std::string image_path = absolute_dir_path + (boost::format("/%08d.png") % index.index_x_).str();
    return SaveImageFile(matrix, image_path);
}

bool ImageFileGenerator::SaveImageFile(const GridMatrix& matrix, std::string image_path) {
    cv::Mat img = std::move(GetImageMatrix(matrix));
    AINFO << "Save path: " << image_path;
    bool success = cv::imwrite(image_path, img);

    if (!success) {
        AERROR << "save image failed to " << image_path;
        return false;
    }
    return success;
}

cv::Mat ImageFileGenerator::GetImageMatrix(const GridMatrix& matrix) const {
    int cols = matrix.get_cols(), rows = matrix.get_rows();
    cv::Mat cvGray(cols, rows, CV_8UC1, cv::Scalar(255));
    for (unsigned int x = 0; x < static_cast<uint32_t>(rows); ++x) {
        for (unsigned int y = 0; y < static_cast<uint32_t>(cols); ++y) {
            const GridCell& map_cell = matrix.get_const_map_cell(x, y);
            const GridCellSingle& cell = map_cell.map_cells_[0];

            unsigned char max_gray_scale_value = 255, min_gray_scale_value = 0;
            unsigned char pixel = cell.intensity_;
            if (pixel_quantizer_) {
                pixel = pixel_quantizer_->Convert(cell);
            }

            if (cell.count_ > 0) {
                if (pixel >= max_gray_scale_value) {
                    cvGray.at<uchar>(1023 - y, x) = static_cast<unsigned char>(max_gray_scale_value);
                } else if (pixel <= min_gray_scale_value) {
                    cvGray.at<uchar>(1023 - y, x) = static_cast<unsigned char>(min_gray_scale_value);
                } else {
                    cvGray.at<uchar>(1023 - y, x) = static_cast<unsigned char>(pixel);
                }
            } else {
                cvGray.at<uchar>(1023 - y, x) = static_cast<unsigned char>(min_gray_scale_value);
            }
        }
    }
    cv::Mat cvRGB(cols, rows, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::cvtColor(cvGray, cvRGB, cv::COLOR_GRAY2BGR);

    for (unsigned int x = 0; x < static_cast<uint32_t>(rows); ++x) {
        for (unsigned int y = 0; y < static_cast<uint32_t>(cols); ++y) {
            const GridCell& map_cell = matrix.get_const_map_cell(x, y);
            const GridCellSingle& cell = map_cell.map_cells_[0];
            if (vertical_segmentation_ && vertical_segmentation_->Segmentation(cell)) {
                cv::Vec3b now_pixel = {0, 255, 0};
                cvRGB.at<cv::Vec3b>(1023 - y, x) = now_pixel;
            }
        }
    }
    return cvRGB;
}

}
}
