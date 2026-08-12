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

#include <cmath>
#include "grid_map.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "cyber/cyber.h"

namespace apollo {
namespace tile_map_images_creator {

GridMap::GridMap(std::shared_ptr<GridOption> option, ImagesCreatorConf conf) {
    option_ = option;
    io_conf_ = conf.input_output_conf();
    resolution_ = option->matrix_resolution;
    resolution_id_ = option->resolution_id;

    if (io_conf_.use_lru_cache()) {
        lru_cache_.Init(io_conf_.lru_cache_size());
    }

    AINFO << "lru cache config on map " << option_->resolution_id
          << "; status=" << std::string(io_conf_.use_lru_cache() ? "true" : "false")
          << ", cache size = " << io_conf_.lru_cache_size();

    auto intensity_converter
            = std::make_shared<IntensityConverter>(conf.point_cloud_processing_conf().intensity_converter_conf());

    std::shared_ptr<VerticalSegmentation> vertical_segmentation = nullptr;
    if (conf.point_cloud_processing_conf().has_vertical_segmentation()) {
        //todo 改名字，加conf
        vertical_segmentation = std::make_shared<VerticalSegmentation>(conf.point_cloud_processing_conf().vertical_segmentation());
    }

    image_file_generator_ = std::make_shared<ImageFileGenerator>();
    image_file_generator_->SetPixelIntensityQuantizer(intensity_converter);
    image_file_generator_->SetVerticalSegmentation(vertical_segmentation);
    image_file_generator_->SetImageDirPath(io_conf_.images_output_dir());
}

GridMap::~GridMap() {
    std::map<MatrixIndex, std::shared_ptr<GridMatrix>>::iterator iter;
    for (iter = matrixs_.begin(); iter != matrixs_.end(); ++iter) {
        iter->second.reset();
    }
    std::cerr << "destruct grid map..." << std::endl;
}

bool GridMap::SetValue(
        double x,
        double y,
        double z,
        double z_relative,
        double z_relative_min,
        double z_relative_max,
        unsigned int count,
        double intensity,
        unsigned char r,
        unsigned char g,
        unsigned char b,
        int layer) {
    const double matrix_resolution = option_->resolution * option_->matrix_size;
    int matrix_index_x = std::floor(x / matrix_resolution);
    int matrix_index_y = std::floor(y / matrix_resolution);
    MatrixIndex matrix_index(matrix_index_x, matrix_index_y);
    // AINFO << "SetValue: matrix_index_x = " << matrix_index_x
    //       << ", matrix_index_y = " << matrix_index_y
    //       << ", x = " << x
    //       << ", y = " << y
    //       << ", z = " << z
    //       << ", z_relative = " << z_relative
    //       << ", z_relative_min = " << z_relative_min
    //       << ", z_relative_max = " << z_relative_max
    //       << ", count = " << count
    //       << ", intensity = " << intensity
    //       << ", r = " << static_cast<int>(r)
    //       << ", g = " << static_cast<int>(g)
    //       << ", b = " << static_cast<int>(b)
    //       << ", layer = " << layer
    //       << ", resolution_id = " << option_->resolution_id;

    total_query_num_++;
    // 如果cache已满，则将最早的瓦片从cache弹出并持久化
    if (io_conf_.use_lru_cache() && !lru_cache_.Update(matrix_index)) {
        MatrixIndex erase_index;
        if (!lru_cache_.Pop(erase_index)) {
            AERROR << "cache pop error :" << boost::format("(%d, %d)") % erase_index.index_x_ % erase_index.index_y_;
        }
        if (!SaveMatrixToDisk(erase_index)) {
            AERROR << "save value to disk error : "
                   << boost::format("(%d, %d)") % erase_index.index_x_ % erase_index.index_y_;
            return false;
        }
        matrixs_[erase_index]->Reset();
        matrixs_.erase(erase_index);
        if (!lru_cache_.Update(matrix_index)) {
            AERROR << "cache update error: " << boost::format("(%d, %d)") % erase_index.index_x_ % erase_index.index_y_;
        }
        AINFO << boost::format("LRU cache full size = %d , pop matrix_index (%d, %d)") % lru_cache_.Size()
                        % erase_index.index_x_ % erase_index.index_y_;

        total_miss_num_++;
        AINFO << "total miss rate : " << total_miss_num_ << "/" << total_query_num_;
    }

    // 寻找现在是否有存储瓦片，如果没存储则创建新瓦片，并尝试从硬盘读取瓦片。
    bool find_matrix = matrixs_.count(matrix_index) > 0;
    if (!find_matrix) {
        std::shared_ptr<GridMatrix> matrix_ptr = std::make_shared<GridMatrix>(option_);
        matrix_ptr->set_index(matrix_index);
        matrixs_[matrix_index] = matrix_ptr;

        if (io_conf_.use_lru_cache() && LoadValueFromDisk(matrix_index)) {
            AINFO << "LRU cache load, x = " << matrix_index.index_x_ << ", y = " << matrix_index.index_y_;
        } else {
            AWARN << "new matrix create, x = " << matrix_index.index_x_ << ", y = " << matrix_index.index_y_;
        }
    }
    matrixs_[matrix_index]->SetValue(x, y, z, z_relative, z_relative_min, z_relative_max, count, intensity, r, g, b, layer);
    return true;
}

bool GridMap::SaveMatrixToDisk(MatrixIndex matrix_index) {
    if (!matrixs_.count(matrix_index)) {
        AERROR << boost::format("save error: no index (%d, %d) in map, res_id = %d") % matrix_index.index_x_
                        % matrix_index.index_y_ % option_->resolution_id;
        return false;
    }
    std::string tile_output_dir = io_conf_.bin_output_dir();

    if (!matrixs_[matrix_index]->SaveBinaryMatrixFile(tile_output_dir)) {
        AERROR << "binary file save fail, filepath = " << tile_output_dir;
        return false;
    }
    AINFO << "tile save, " << boost::format("(%d, %d)") % matrix_index.index_x_ % matrix_index.index_y_;

    if (!image_file_generator_->SaveImageFile(*matrixs_[matrix_index])) {
        AERROR << "image file save fail, file dir = " << io_conf_.images_output_dir();
        return false;
    }
    AINFO << "image save, " << boost::format("(%d, %d)") % matrix_index.index_x_ % matrix_index.index_y_;

    return true;
}

bool GridMap::LoadValueFromDisk(MatrixIndex matrix_index) {
    std::string tile_output_dir = io_conf_.bin_output_dir();

    if (!boost::filesystem::exists(tile_output_dir) || !boost::filesystem::is_directory(tile_output_dir)) {
        AERROR << "binary directory not exist : " << tile_output_dir;
        return false;
    }

    if (!matrixs_.count(matrix_index)) {
        AERROR << boost::format("index (%d, %d) not exists in map") % matrix_index.index_x_ % matrix_index.index_y_;
        return false;
    }

    std::string binary_file_path = tile_output_dir + "/" + matrixs_[matrix_index]->GetMatrixKey() + ".bin";
    if (!boost::filesystem::exists(binary_file_path)) {
        AINFO << boost::format("binary file not found for index (%d, %d), %s not exist") % matrix_index.index_x_
                        % matrix_index.index_y_ % binary_file_path;
        return false;
    }

    if (!matrixs_[matrix_index]->LoadBinaryMatrixFile(tile_output_dir)) {
        AERROR << "load matrix fail, " << tile_output_dir;
        return false;
    }
    AINFO << "matrix load, " << boost::format("(%d, %d)") % matrix_index.index_x_ % matrix_index.index_y_;

    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
