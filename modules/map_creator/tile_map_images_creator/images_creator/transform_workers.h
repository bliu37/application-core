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

#include <chrono>

#include "boost/thread/thread_pool.hpp"

#include "cyber/base/bounded_queue.h"
#include "cyber/base/thread_pool.h"
#include "cyber/base/thread_safe_queue.h"
#include "modules/map_creator/tile_map_images_creator/common/frame.h"
#include "modules/map_creator/tile_map_images_creator/coord_transformer/frame_transformer.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_renderer.h"

namespace apollo {
namespace tile_map_images_creator {
class TransformWorkers {
public:
    TransformWorkers() = default;
    TransformWorkers(const ImagesCreatorConf& conf) : conf_(conf) {}

    void Start() {
        dispatch_future_ = std::async(std::launch::async, &TransformWorkers::Dispatch, this);
        render_future_ = std::async(std::launch::async, &TransformWorkers::Render, this);
    }

    bool Init(const std::shared_ptr<ImageRenderer>& image_renderer, int worker_num);

    bool AddTransformTask(const ImageCreatorFrame& frame);

    void WaitForFinished();

    void SetFinishedOnException();

private:
    void TransformWorker(
            const ImageCreatorFrame& source_frame,
            std::shared_ptr<std::promise<ResultFrame>>& result_promise);

    // 单独线程
    ImagesCreatorResult Dispatch();

    // 单独线程
    ImagesCreatorResult Render();

    cyber::base::BoundedQueue<ImageCreatorFrame> input_frames_;
    cyber::base::BoundedQueue<std::shared_ptr<std::future<ResultFrame>>> result_frames_future_;
    std::unique_ptr<cyber::base::ThreadPool> worker_pool_;

    std::shared_ptr<FrameTransformer> frame_transformer_ = nullptr;
    std::shared_ptr<ImageRenderer> image_renderer_ = nullptr;
    std::atomic_bool exception_finished_ = {false}, task_finished_ = {false};

    std::future<ImagesCreatorResult> dispatch_future_, render_future_;

    int thread_pool_workers_ = 4;

    ImagesCreatorConf conf_;
};
}  // namespace tile_map_images_creator
}  // namespace apollo
