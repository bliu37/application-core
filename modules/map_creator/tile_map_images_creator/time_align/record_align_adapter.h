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
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/time_align_cache.hpp"

namespace apollo {
namespace tile_map_images_creator {

/**
 * @brief The definition of RecordAlignAdapter. Requiring templates should be the type of aligned proto message in
 * records. The first element in template definition should be the key message.
 */
template <typename... Args>
class RecordAlignAdapter;

template <>
class RecordAlignAdapter<> {
public:
    RecordAlignAdapter() = default;

    virtual ~RecordAlignAdapter() = default;

    RecordAlignAdapter(const std::vector<std::string>& record_files_path);

    void SetChannels() {}

    void SetTimestampFunctor() {}

    virtual bool MessageCategorization(const std::shared_ptr<cyber::record::RecordMessage>& message) {
        return true;
    }

    bool GetNextFrameWithTimestamp(std::tuple<>& frame, uint64_t timestamp) {
        frame = std::make_tuple<>();
        return true;
    }

    /**
     * @brief Read the next message from the records. If the current record has been fully read, proceed to read the
     * first message of the next record.
     * @return return true if read successfully, false if read to the end.
     */
    bool LoadNextRecordMessage();

protected:
    bool finished_;
    std::vector<std::string> record_files_path_;
    unsigned int reader_pointer_now_;
    std::shared_ptr<cyber::record::RecordReader> reader_now_;
    std::shared_ptr<cyber::record::RecordMessage> record_message_now_;
};

template <typename T, typename... Rest>
class RecordAlignAdapter<T, Rest...> : public RecordAlignAdapter<Rest...> {
public:
    RecordAlignAdapter() = default;

    RecordAlignAdapter(const std::vector<std::string>& record_files_path) :
            RecordAlignAdapter<Rest...>(record_files_path) {}

    virtual ~RecordAlignAdapter() = default;

    /**
     * @brief set channels corresponding to template
     */
    template <typename... Args>
    void SetChannels(const std::string& channel, const Args&... rest_channels);

    /**
     * @brief set the method of reading timestamp corresponding type of each channel message.
     * @param the input functions should return timestamp in 64 bit unsigned integer format.
     */
    void SetTimestampFunctor(
            std::function<uint64_t(std::shared_ptr<T>)> functor,
            std::function<uint64_t(std::shared_ptr<Rest>)>... rest_functor);

    /**
     * @brief Get the next time aligned frame. The result contain the next key message
     *        (the first parameter defined in the template), and other type message closest to the key message timestamp
     * @return frame. return time aligned frames in tuple format
     * @return aligned_timestamp. return the timestamp of the key message
     * @return bool. return true if read successfully, false if read to the end.
     */
    bool GetNextFrame(std::tuple<std::shared_ptr<T>, std::shared_ptr<Rest>...>& frame, uint64_t& aligned_timestamp);

    /**
     * @brief Get messages closest to the provided timestamp.
     * @return frame. return time aligned frames in tuple format
     * @param timestamp. provided timestamp
     * @return bool. return true if read successfully, false if read to the end.
     * @note When calling multiple times, the timestamp provided by the next call needs to be larger than the previous
     * one
     */
    bool GetNextFrameWithTimestamp(
            std::tuple<std::shared_ptr<T>, std::shared_ptr<Rest>...>& frame,
            const u_int64_t timestamp);

    // the message handle and its corresponding cache.
    std::string channel_name_;
    std::shared_ptr<TimeAlignCache<T>> time_align_cache_ = nullptr;

protected:
    /**
     * @brief Assign new record message to the corresponding cache based on the channel of new type.
     */
    virtual bool MessageCategorization(const std::shared_ptr<cyber::record::RecordMessage>& message);

private:
    void PopPrimaryMessage();

    bool EnsurePrimaryMessage(uint64_t& primary_timestamp);
};

template <typename T, typename... Rest>
void RecordAlignAdapter<T, Rest...>::SetTimestampFunctor(
        std::function<uint64_t(std::shared_ptr<T>)> functor,
        std::function<uint64_t(std::shared_ptr<Rest>)>... rest_functor) {
    time_align_cache_ = std::make_shared<TimeAlignCache<T>>(functor);
    time_align_cache_->Init();
    RecordAlignAdapter<Rest...>::SetTimestampFunctor(rest_functor...);
}

template <typename T, typename... Rest>
template <typename... Args>
void RecordAlignAdapter<T, Rest...>::SetChannels(const std::string& channel, const Args&... rest_channels) {
    channel_name_ = channel;
    RecordAlignAdapter<Rest...>::SetChannels(rest_channels...);
}

template <typename T, typename... Rest>
void RecordAlignAdapter<T, Rest...>::PopPrimaryMessage() {
    if (time_align_cache_->HeadCheck()) {
        std::shared_ptr<T> front_ele;
        time_align_cache_->CachePop(front_ele);
    }
}

template <typename T, typename... Rest>
bool RecordAlignAdapter<T, Rest...>::EnsurePrimaryMessage(uint64_t& primary_timestamp) {
    while (!time_align_cache_->HeadCheck()) {
        if (!this->LoadNextRecordMessage()) {
            AINFO << "records load finish";
            return false;
        }
        MessageCategorization(this->record_message_now_);
    }

    primary_timestamp = this->time_align_cache_->HeadTimestamp();
    return true;
}

template <typename T, typename... Rest>
bool RecordAlignAdapter<T, Rest...>::MessageCategorization(
        const std::shared_ptr<cyber::record::RecordMessage>& message) {
    if (channel_name_ == message->channel_name) {
        std::shared_ptr<T> instantiation_message = std::make_shared<T>();
        if (instantiation_message->ParseFromString(message->content)) {
            time_align_cache_->Insert(instantiation_message);
            return true;
        } else {
            AERROR << "message with channel = " << channel_name_ << " parse error";
            return false;
        }
    } else {
        return RecordAlignAdapter<Rest...>::MessageCategorization(message);
    }
}

template <typename T, typename... Rest>
bool RecordAlignAdapter<T, Rest...>::GetNextFrame(
        std::tuple<std::shared_ptr<T>, std::shared_ptr<Rest>...>& frame,
        uint64_t& aligned_timestamp) {
    while (!this->finished_) {
        (void)PopPrimaryMessage();
        uint64_t primary_message_timestamp = 0;
        if (!EnsurePrimaryMessage(primary_message_timestamp)) {
            AINFO << "record load to the end, finish";
            return false;
        }

        if (!GetNextFrameWithTimestamp(frame, primary_message_timestamp)) {
            AINFO << "get next frame with timestamp fail, current primary frame discard, primary message timestamp = "
                  << primary_message_timestamp;
            continue;
        }
        aligned_timestamp = primary_message_timestamp;
        return true;
    }
    AINFO << "record reach finished";
    return false;
}

template <typename T, typename... Rest>
bool RecordAlignAdapter<T, Rest...>::GetNextFrameWithTimestamp(
        std::tuple<std::shared_ptr<T>, std::shared_ptr<Rest>...>& frame,
        const u_int64_t timestamp) {
    std::tuple<std::shared_ptr<Rest>...> rest_frame;

    if (!RecordAlignAdapter<Rest...>::GetNextFrameWithTimestamp(rest_frame, timestamp)) {
        AERROR << "timestamp " << timestamp << " bind fail";
        return false;
    }

    std::shared_ptr<T> time_align_element = std::make_shared<T>();
    while (!time_align_cache_->FindNearest(timestamp, time_align_element)) {
        if (timestamp < time_align_cache_->HeadTimestamp()) {
            AINFO << boost::format(
                             "Timestamp = %llu invalid, earlier than all timestamp in cache, the earliest one = %llu")
                            % timestamp % time_align_cache_->HeadTimestamp();
            return false;
        }

        if (!this->LoadNextRecordMessage()) {
            AINFO << "records load finish";
            return false;
        }
        MessageCategorization(this->record_message_now_);
    }
    frame = std::tuple_cat(std::make_tuple(time_align_element), rest_frame);
    ADEBUG << "channel " << channel_name_ << " bind; " << time_align_cache_->HeadTimestamp();
    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
