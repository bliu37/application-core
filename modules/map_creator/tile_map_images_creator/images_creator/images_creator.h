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
#include <memory>
#include <mutex>
#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"

#include "modules/map_creator/tile_map_images_creator/coord_transformer/frame_transformer.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_creator_result.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_frame_producer.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_renderer.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/pcd_creator.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/transform_workers.h"
#include "modules/map_creator/tile_map_images_creator/matrix_generator/matrix_generator.h"
#include "modules/map_creator/tile_map_images_creator/point_cloud_filter/point_cloud_preprocessing_filter.h"
#include "modules/map_creator/tile_map_images_creator/tiles_creator/tiles_creator.h"

/**
 * @namespace apollo::tile_map_images_creator
 * @brief apollo::tile_map_images_creator
 */
namespace apollo {
namespace tile_map_images_creator {

using ResultFuture = std::shared_future<ImagesCreatorResult>;

/**
 * @brief 底图生成类
 * 使用方法：
 *     1. 首先传入ImagesCreatorConf配置创建类对象
 *     2. 然后调用Start()方法获取异步结果对象
 *     3. 根据异步对象判断是否完成，调用GetProgress获取处理进度
 */
class ImagesCreator {
public:
    explicit ImagesCreator(const ImagesCreatorConf& conf) : conf_(conf){};

    /**
     * @brief 启动底图生成程序
     *
     * @return 返回一个future对象，格式为ImagesCreatorResult
     */
    ResultFuture Start();

    /**
     * @brief 获取底图生成进度
     */
    float GetProgress();

private:
    bool Init(std::string* err_message);
    bool GetRecordFiles(std::string* err_message);
    bool GetTotalMessageNum(std::string* err_message);
    ImagesCreatorResult Run();

    ImagesCreatorConf conf_;
    ImagesCreatorResult result_;
    std::vector<std::string> record_files_;

    std::shared_ptr<ImageRenderer> image_renderer_ = nullptr;

    std::unique_ptr<ImageFrameProducer> image_frame_producer = nullptr;

    std::unique_ptr<TransformWorkers> transform_workers = nullptr;

    uint64_t total_message_num_ = 0;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
