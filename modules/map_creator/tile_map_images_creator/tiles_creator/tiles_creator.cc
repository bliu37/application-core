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
#include "modules/map_creator/tile_map_images_creator/tiles_creator/tiles_creator.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include "cyber/record/record_reader.h"
#include "modules/map_creator/tile_map_images_creator/common/macro.h"

namespace apollo {
namespace tile_map_images_creator {

TileResultFuture TilesCreator::Start() {
    return std::async(std::launch::async, &TilesCreator::Run, this);
}

TilesCreatorResult TilesCreator::Run() {
    TilesCreatorResult result;
    result.succeeded = false;
    if (!Init(&result.message)) {
        return result;
    }

    AINFO << "begin read image info...";
    for (const auto& tile_image : base_tile_images_) {
        AINFO << "read image: " << tile_image.path;
        auto img = cv::imread(tile_image.path, cv::IMREAD_GRAYSCALE);
        if (!img.data) {
            result.message = "read image failed: " + tile_image.path;
            AERROR << result.message;
            return result;
        }
        for (auto& iter : tiles_) {
            const int& level = iter.first;
            NODES& nodes = iter.second;
            size_t scale = 1 << (level_ - level);
            size_t img_scale = img.rows / scale;
            cv::Mat scale_mat;
            cv::resize(img, scale_mat, cv::Size(img_scale, img_scale));
            size_t y_index = tile_image.y / scale;
            size_t x_index = tile_image.x / scale;
            if (nodes.find(y_index) == nodes.end()) {
                nodes.emplace(y_index, GRIDS());
            }
            GRIDS& node = nodes[y_index];
            if (node.find(x_index) == node.end()) {
                node.emplace(x_index, cv::Mat::zeros(cv::Size(img.rows, img.cols), CV_8UC1));
            }

            cv::Mat& mat = node[x_index];
            size_t offset_y = img.rows - (tile_image.y % scale) * img_scale - img_scale;
            size_t offset_x = (tile_image.x % scale) * img_scale;
            for (uint16_t row = 0; row < img_scale; ++row) {
                for (uint16_t col = 0; col < img_scale; ++col) {
                    if (scale_mat.at<uchar>(row, col) == 0) {
                        continue;
                    }
                    mat.at<uchar>(offset_y + row, offset_x + col) = scale_mat.at<uchar>(row, col);
                }
            }
        }
        ForwardProgress();
    }
    AINFO << "finish read image info.";

    if (!WriteImages(&result.message)) {
        return result;
    }

    result.succeeded = true;
    return result;
}

bool TilesCreator::WriteImages(std::string* err_message) {
    AINFO << "begin create tile images";
    for (const auto& iter0 : tiles_) {
        auto& level = iter0.first;
        for (const auto& iter1 : iter0.second) {
            auto& y_index = iter1.first;
            std::string target_dir = (boost::format("%s/%d/%08d") % base_dir_ % level % y_index).str();
            if (!apollo::cyber::common::EnsureDirectory(target_dir)) {
                WRITE_ERR_MESSAGE("create target dir failed: " + target_dir);
            }
            for (const auto& iter2 : iter1.second) {
                auto& x_index = iter2.first;
                auto& mat = iter2.second;
                std::string image_file = (boost::format("%s/%08d.png") % target_dir % x_index).str();
                if (!cv::imwrite(image_file, mat)) {
                    WRITE_ERR_MESSAGE("write image failed: " + image_file);
                }
            }
        }
    }
    AINFO << "finish create tile images";
    return true;
}

bool TilesCreator::Init(std::string* err_message) {
    if (level_ <= 0) {
        WRITE_ERR_MESSAGE("level must be greater than 0.");
    }
    for (int i = 0; i < level_; ++i) {
        tiles_.emplace(i, NODES());
    }

    if (!cyber::common::DirectoryExists(base_dir_)) {
        WRITE_ERR_MESSAGE("base dir [" + base_dir_ + "] not exists.");
    }

    std::string buttom_dir = (boost::format("%s/%d") % base_dir_ % level_).str();
    if (!cyber::common::DirectoryExists(buttom_dir)) {
        WRITE_ERR_MESSAGE("buttom level images dir [" + buttom_dir + "] not exists.");
    }

    for (const auto& path : cyber::common::Glob(buttom_dir + "/[0-9]*/[0-9]*.png")) {
        auto x_pos = path.rfind('/');
        auto y_pos = path.rfind('/', x_pos - 1);
        TileImage ti;
        ti.y = std::stoi(path.substr(y_pos + 1, x_pos - y_pos - 1));
        ti.x = std::stoi(path.substr(x_pos + 1, path.size() - x_pos - 1 - 4));
        ti.path = path;
        base_tile_images_.push_back(std::move(ti));
    }
    if (base_tile_images_.empty()) {
        WRITE_ERR_MESSAGE("no images found in " + buttom_dir);
    }
    return true;
}

void TilesCreator::ForwardProgress() {
    ++processed_images_num_;
    progress_ = 100.0 * processed_images_num_ / base_tile_images_.size();
}

}  // namespace tile_map_images_creator
}  // namespace apollo
