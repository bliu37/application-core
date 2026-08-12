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

#include "modules/map_creator/tile_map_images_creator/images_creator/transform_workers.h"

#include <thread>

namespace apollo {
namespace tile_map_images_creator {

bool TransformWorkers::Init(const std::shared_ptr<ImageRenderer>& image_renderer, int worker_num) {
    RETURN_VAL_IF(!input_frames_.Init(500, new cyber::base::BlockWaitStrategy()), false);
    RETURN_VAL_IF(!result_frames_future_.Init(500, new cyber::base::BlockWaitStrategy()), false);

    // todo 接口格式可能被更改，目前slam模式给的是IMU坐标pose
    if (conf_.has_slam_mode_selection_conf() && conf_.slam_mode_selection_conf().enable_slam_mode()) {
        StaticTransformer::Instance()->SetLocalizationFrameId("imu");
    } else {
        StaticTransformer::Instance()->SetLocalizationFrameId("localization");
    }

    frame_transformer_ = std::make_unique<FrameTransformer>();
    image_renderer_ = image_renderer;
    thread_pool_workers_ = worker_num;
    worker_pool_ = std::make_unique<cyber::base::ThreadPool>(worker_num);
    return true;
}

bool TransformWorkers::AddTransformTask(const ImageCreatorFrame& frame) {
    ADEBUG << "source frame enqueue, Waiting... size = " << input_frames_.Size();
    if (input_frames_.Size() > static_cast<uint32_t>(thread_pool_workers_ * 3)) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    if (!input_frames_.Enqueue(frame)) {
        (void)SetFinishedOnException();
        return false;
    }
    ADEBUG << "source frame enqueue, size = " << input_frames_.Size();
    return true;
}

void TransformWorkers::WaitForFinished() {
    while ((!cyber::IsShutdown() && !exception_finished_)
           && (input_frames_.Size() > 0 || result_frames_future_.Size() > 0)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    task_finished_ = true;
    input_frames_.BreakAllWait();
    result_frames_future_.BreakAllWait();

    dispatch_future_.wait();
    render_future_.wait();
}

void TransformWorkers::TransformWorker(
        const ImageCreatorFrame& source_frame,
        std::shared_ptr<std::promise<ResultFrame>>& result_promise) {
    ResultFrame result_frame;
    if (cyber::IsShutdown() || exception_finished_ || !frame_transformer_->Transform(source_frame, result_frame)) {
        (void)SetFinishedOnException();
        AERROR << "error occur on transform worker, pcd sequence num = "
               << source_frame.pcd_frame_->header().sequence_num();
    }

    result_promise->set_value(std::move(result_frame));
}

ImagesCreatorResult TransformWorkers::Dispatch() {
    ImagesCreatorResult dispatch_process_result;
    while (!cyber::IsShutdown() && !exception_finished_) {
        ImageCreatorFrame frame;
        if (!input_frames_.WaitDequeue(&frame)) {
            if (task_finished_) {
                AINFO << "dispatch task finished, returned";
                dispatch_process_result.succeeded = true;
                return dispatch_process_result;
            }

            dispatch_process_result.succeeded = false;
            dispatch_process_result.message = "error occur on input_frames_.WaitDequeue";
            (void)SetFinishedOnException();
            AERROR << "error occur on input_frames_.WaitDequeue";
            return dispatch_process_result;
        }

        ADEBUG << "source frame dequeue, size = " << input_frames_.Size();

        std::shared_ptr<std::promise<ResultFrame>> result_promise = std::make_shared<std::promise<ResultFrame>>();
        std::future<ResultFrame> result_future = result_promise->get_future();

        ADEBUG << "Waiting for result frame enqueue, size = " << result_frames_future_.Size();
        if (result_frames_future_.Size() > static_cast<uint32_t>(thread_pool_workers_ * 3)) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        if (!result_frames_future_.Enqueue(std::make_unique<std::future<ResultFrame>>(std::move(result_future)))) {
            dispatch_process_result.succeeded = false;
            dispatch_process_result.message = "error occur on result_frames_future_.Enqueue";
            (void)SetFinishedOnException();
            AERROR << "error occur on result_frames_future_.Enqueue";
            return dispatch_process_result;
        }
        ADEBUG << "result frame future enqueue, size = " << result_frames_future_.Size();

        // todo 考虑限制排队等待的任务数，降低在计算性能低下时的内存压力
        worker_pool_->Enqueue(&TransformWorkers::TransformWorker, this, frame, result_promise);
    }
    dispatch_process_result.succeeded = true;
    return dispatch_process_result;
}

ImagesCreatorResult TransformWorkers::Render() {
    ImagesCreatorResult render_process_result;
    while (!cyber::IsShutdown() && !exception_finished_) {
        std::shared_ptr<std::future<ResultFrame>> result_future;
        if (!result_frames_future_.WaitDequeue(&result_future)) {
            if (task_finished_) {
                ADEBUG << "render task finished, returned";
                render_process_result.succeeded = true;
                return render_process_result;
            }

            render_process_result.succeeded = false;
            render_process_result.message = "error occur on result_frames_future_.WaitDequeue";
            SetFinishedOnException();
            AERROR << "error occur on result_frames_future_.WaitDequeue";
            return render_process_result;
        }
        AINFO << "result frame future dequeue, size = " << result_frames_future_.Size()
              << ". Waiting for transforming...";

        ResultFrame frame = std::move(result_future->get());
        if (cyber::IsShutdown() || exception_finished_) {
            render_process_result.succeeded = false;
            return render_process_result;
        }

        // 渲染
        render_process_result = image_renderer_->AppendResultFrame(frame);
        if (!render_process_result.succeeded) {
            SetFinishedOnException();
            return render_process_result;
        }
    }
    render_process_result.succeeded = true;
    return render_process_result;
}

void TransformWorkers::SetFinishedOnException() {
    exception_finished_ = true;
    input_frames_.BreakAllWait();
    result_frames_future_.BreakAllWait();
}

}  // namespace tile_map_images_creator
}  // namespace apollo
