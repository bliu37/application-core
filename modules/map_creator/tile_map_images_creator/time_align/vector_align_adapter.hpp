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

#include <vector>

#include "boost/format.hpp"

#include "cyber/cyber.h"
#include "modules/map_creator/tile_map_images_creator/time_align/time_align_cache.hpp"

namespace apollo {
namespace tile_map_images_creator {

/**
 * @brief time align adapter for messages in vector type.
 */
template <typename T>
class VectorAlignAdapter {
public:
    VectorAlignAdapter() : pointer_now_(0), finished_(false) {}

    void Init(const std::vector<std::shared_ptr<T>>& vec_frames);

    void SetTimestampFunctor(std::function<uint64_t(std::shared_ptr<T>)> functor);

    /**
     * @brief find the nearest message of the provided timestamp.
     * @return frame. tuple format message of result frame
     * @param timestamp. provided timestamp
     * @note When calling multiple times, the timestamp provided by the next call needs to be larger than the previous
     * one
     */
    bool GetNextFrameWithTimestamp(std::tuple<std::shared_ptr<T>>& frame, const u_int64_t timestamp);

private:
    std::vector<std::shared_ptr<T>> element_vector_;
    unsigned int pointer_now_;
    bool finished_;

    std::function<uint64_t(std::shared_ptr<T>)> timestamp_method_;
    std::shared_ptr<TimeAlignCache<T>> time_align_cache_ = nullptr;
};

template <typename T>
void VectorAlignAdapter<T>::SetTimestampFunctor(std::function<uint64_t(std::shared_ptr<T>)> functor) {
    timestamp_method_ = functor;
    time_align_cache_ = std::make_shared<TimeAlignCache<T>>(timestamp_method_);
    time_align_cache_->Init();
}

template <typename T>
void VectorAlignAdapter<T>::Init(const std::vector<std::shared_ptr<T>>& vec_frames) {
    pointer_now_ = 0;
    finished_ = vec_frames.empty();
    element_vector_.assign(vec_frames.begin(), vec_frames.end());
    AINFO << "vector align adapter inited, element num = " << element_vector_.size();
}

template <typename T>
bool VectorAlignAdapter<T>::GetNextFrameWithTimestamp(
        std::tuple<std::shared_ptr<T>>& frame,
        const u_int64_t timestamp) {
    std::shared_ptr<T> time_align_element = nullptr;
    while (!time_align_cache_->FindNearest(timestamp, time_align_element)) {
        if (finished_) {
            return false;
        }

        if (timestamp < time_align_cache_->HeadTimestamp()) {
            AINFO << boost::format(
                             "Timestamp = %llu invalid, earlier than all timestamp in cache, the earliest one = %llu")
                            % timestamp % time_align_cache_->HeadTimestamp();
            return false;
        }

        time_align_cache_->Insert(element_vector_[pointer_now_]);

        if (++pointer_now_ >= element_vector_.size()) {
            finished_ = true;
        }
    }
    frame = std::make_tuple(time_align_element);
    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
