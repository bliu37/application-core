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

#include "cyber/cyber.h"

namespace apollo {
namespace tile_map_images_creator {

/**
 * @brief cache for time align
 */
template <typename T>
class TimeAlignCache {
public:
    TimeAlignCache(const std::function<uint64_t(std::shared_ptr<T>)>& timestamp_method) :
            timestamp_method_(timestamp_method) {}

    TimeAlignCache(){};

    bool Init();

    void Insert(const std::shared_ptr<T>& element_ptr);

    /**
     * @brief find the nearest timestamp of the provided one in the cache queue.
     * @note When calling multiple times, the timestamp provided by the next call needs to be larger than the previous
     * one
     */
    bool FindNearest(uint64_t timestamp, std::shared_ptr<T>& nearest);

    bool CachePop(std::shared_ptr<T>& pop_value);

    u_int64_t HeadTimestamp();

    bool HeadCheck();

    bool HeadAndCacheCheck();

private:
    std::function<uint64_t(std::shared_ptr<T>)> timestamp_method_ = nullptr;
    // todo 增加时间插值函数

    std::shared_ptr<T> head_ = nullptr;
    std::deque<std::shared_ptr<T>> cache_queue;
};

template <typename T>
bool TimeAlignCache<T>::Init() {
    AINFO << __FUNCTION__;

    head_ = nullptr;
    cache_queue.clear();
    return true;
}

template <typename T>
void TimeAlignCache<T>::Insert(const std::shared_ptr<T>& element_ptr) {
    if (head_ == nullptr) {
        head_ = element_ptr;
    } else {
        cache_queue.push_back(element_ptr);
    }
}

template <typename T>
bool TimeAlignCache<T>::FindNearest(uint64_t timestamp, std::shared_ptr<T>& nearest) {
    if (head_ == nullptr) {
        AERROR << "head_ null for timestamp, now =" << timestamp;
        return false;
    }

    if (cache_queue.empty()) {
        ADEBUG << "cache queue error for timestamp, but head_ is valid, now = " << timestamp;
        return false;
    }

    // todo 此数据为无效数据，应当区分结束和无效
    if (timestamp < timestamp_method_(head_)) {
        AERROR << "timestamp earlier than all element in cache, with now = " << timestamp
               << "; head_ = " << timestamp_method_(head_);
        return false;
    }

    while (!cache_queue.empty()
           && !(timestamp >= timestamp_method_(head_) && timestamp <= timestamp_method_(cache_queue.front()))) {
        std::shared_ptr<T> temp;
        CachePop(temp);
    }

    if (cache_queue.empty()) {
        ADEBUG << "Cannot find correspond nearest unit until cache empty";
        return false;
    } else {
        // 找到t的两个相邻数据，选取距离较近的一个返回
        ADEBUG << "Unit Found ! head_->timestamp = " << timestamp_method_(head_) << "; now = " << timestamp
               << "; cache front = " << timestamp_method_(cache_queue.front()) << std::endl;
        if (timestamp - timestamp_method_(head_) <= timestamp_method_(cache_queue.front()) - timestamp) {
            nearest = head_;
        } else {
            nearest = cache_queue.front();
        }
        ADEBUG << "Unit Bind ! now = " << timestamp << "; bind = " << timestamp_method_(nearest);
        return true;
    }
}

template <typename T>
bool TimeAlignCache<T>::CachePop(std::shared_ptr<T>& pop_value) {
    if (head_ == nullptr)
        return false;
    pop_value = head_;
    if (cache_queue.empty()) {
        head_ = nullptr;
    } else {
        head_ = cache_queue.front();
        cache_queue.pop_front();
    }
    return true;
}

template <typename T>
u_int64_t TimeAlignCache<T>::HeadTimestamp() {
    if (head_ == nullptr) {
        return 0;
    }
    return timestamp_method_(head_);
}

template <typename T>
bool TimeAlignCache<T>::HeadCheck() {
    return head_ != nullptr;
}

template <typename T>
bool TimeAlignCache<T>::HeadAndCacheCheck() {
    return HeadCheck() && !cache_queue.empty();
}

}  // namespace tile_map_images_creator
}  // namespace apollo
