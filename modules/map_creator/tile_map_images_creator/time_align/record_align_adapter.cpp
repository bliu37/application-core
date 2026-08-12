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
 *****************************************************************************/ \
#include "modules/map_creator/tile_map_images_creator/time_align/record_align_adapter.h"

namespace apollo {
namespace tile_map_images_creator {

RecordAlignAdapter<>::RecordAlignAdapter(const std::vector<std::string>& record_files_path) {
    record_files_path_.assign(record_files_path.begin(), record_files_path.end());
    reader_pointer_now_ = 0;
    if (record_files_path_.empty()) {
        AERROR << "record_files_path should not be empty";
        finished_ = true;
    } else {
        reader_now_ = std::make_shared<cyber::record::RecordReader>(record_files_path_[reader_pointer_now_]);
        AINFO << "read record file " << record_files_path_[reader_pointer_now_] << " start.";
        finished_ = false;
    }
}

bool RecordAlignAdapter<>::LoadNextRecordMessage() {
    if (finished_) {
        return false;
    }

    record_message_now_ = std::make_shared<cyber::record::RecordMessage>();
    while (!reader_now_->ReadMessage(record_message_now_.get())) {
        AINFO << "record file " << record_files_path_[reader_pointer_now_] << " read finished";

        reader_pointer_now_++;
        if (reader_pointer_now_ >= record_files_path_.size()) {
            finished_ = true;
            return false;
        }

        reader_now_ = std::make_shared<cyber::record::RecordReader>(record_files_path_[reader_pointer_now_]);
    }
    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo