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

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

/**
 * @namespace apollo::tile_map_images_creator
 * @brief apollo::tile_map_images_creator
 */
namespace apollo {
namespace tile_map_images_creator {

struct TilesCreatorResult {
    bool succeeded;
    std::string message;
};
using TileResultFuture = std::shared_future<TilesCreatorResult>;

struct TileImage {
    size_t x;
    size_t y;
    std::string path;
};

using GRIDS = std::map<size_t, cv::Mat>;
using NODES = std::map<size_t, GRIDS>;

/**
 * @brief creator all level tile images from buttom tile images directory.
 */
class TilesCreator {
public:
    explicit TilesCreator(const std::string& base_dir, const int level) : base_dir_(base_dir), level_(level){};

    /**
     * @brief Start creator all level tile images.
     */
    TileResultFuture Start();

    /**
     * @brief Get processing progress.
     */
    float GetProgress() {
        return progress_;
    };

private:
    bool Init(std::string* err_message);
    bool WriteImages(std::string* err_message);
    void ForwardProgress();
    TilesCreatorResult Run();

    std::atomic_uint64_t processed_images_num_{0};
    float progress_ = 0.0;

    std::vector<TileImage> base_tile_images_;
    std::map<int, NODES> tiles_;

    std::string base_dir_;
    int level_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
