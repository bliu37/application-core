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

#include "modules/map_creator/tile_map_images_creator/images_creator/images_creator.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include "cyber/record/record_reader.h"
#include "modules/map_creator/tile_map_images_creator/common/macro.h"
#include "modules/map_creator/tile_map_images_creator/common/utils.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/pcd_creator.h"
#include "modules/map_creator/tile_map_images_creator/time_align/multi_source_frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/single_source_frame_loader.h"

namespace apollo {
namespace tile_map_images_creator {

using apollo::common::PointENU;
using apollo::drivers::PointCloud;
using apollo::localization::LocalizationEstimate;

ResultFuture ImagesCreator::Start() {
    return std::async(std::launch::async, &ImagesCreator::Run, this);
}

ImagesCreatorResult ImagesCreator::Run() {
    ImagesCreatorResult result;
    result.succeeded = false;
    if (!Init(&result.message)) {
        return result;
    }

    (void)transform_workers->Start();

    ImageCreatorFrame frame;
    while (!cyber::IsShutdown() && image_frame_producer->LoadNextFrame(frame)) {
        if (!transform_workers->AddTransformTask(frame)) {
            AERROR << "adding transform task fail, localization sequence id = "
                   << frame.localization_frame_->header().sequence_num();
            break;
        }
    }

    (void)transform_workers->WaitForFinished();
    return image_renderer_->FinalOperation();
}

bool ImagesCreator::Init(std::string* err_message) {
    if (!cyber::common::DirectoryExists(conf_.input_output_conf().input_dir())) {
        WRITE_ERR_MESSAGE("input dir [" + conf_.input_output_conf().input_dir() + "] not exists.");
    }
    if (!cyber::common::EnsureDirectory(conf_.input_output_conf().images_output_dir())) {
        WRITE_ERR_MESSAGE("image output dir [" + conf_.input_output_conf().images_output_dir() + "] not exists.");
    }

    if (!cyber::common::EnsureDirectory(conf_.input_output_conf().bin_output_dir())) {
        WRITE_ERR_MESSAGE("bin output dir [" + conf_.input_output_conf().bin_output_dir() + "] not exists.");
    }

    std::string bin_output_dir = conf_.input_output_conf().bin_output_dir();
    if (!cyber::common::RemoveAllFiles(bin_output_dir)) {
        WRITE_ERR_MESSAGE("error occurred when removing files in bin output dir [" + bin_output_dir + "]");
    } else {
        AINFO << boost::format("removing files in bin output dir, path = %s") % bin_output_dir;
    }

    if (!GetRecordFiles(err_message)) {
        return false;
    }
    if (!GetTotalMessageNum(err_message)) {
        return false;
    }
    if (total_message_num_ == 0) {
        WRITE_ERR_MESSAGE("no point cloud message found.");
    }

    image_frame_producer = std::make_unique<ImageFrameProducer>(conf_);
    RETURN_VAL_IF(!image_frame_producer->Init(record_files_, err_message), false);

    image_renderer_ = std::make_shared<ImageRenderer>(conf_);
    RETURN_VAL_IF(!image_renderer_->Init(total_message_num_), false);

    transform_workers = std::make_unique<TransformWorkers>(conf_);
    transform_workers->Init(image_renderer_, conf_.coordinate_transformer_conf().worker_num());

    return true;
}

bool ImagesCreator::GetRecordFiles(std::string* err_message) {
    record_files_.clear();
    auto path = conf_.input_output_conf().input_dir();
    if (path.empty()) {
        WRITE_ERR_MESSAGE("input dir is empty");
    }
    if (path.back() == '/') {
        path += "*";
    } else {
        path += "/*";
    }
    for (auto& filename : cyber::common::Glob(path)) {
        cyber::common::FileType type;
        if (!cyber::common::GetType(filename, &type)) {
            WRITE_ERR_MESSAGE("failed to get file type at " + filename);
        }
        if (boost::filesystem::is_empty(filename)) {
            AWARN << "file[" << filename << "] is empty";
            continue;
        }
        AINFO << "find file name " << filename;
        record_files_.push_back(filename);
    }
    if (record_files_.empty()) {
        WRITE_ERR_MESSAGE("no record file found.");
    }
    return true;
}

bool ImagesCreator::GetTotalMessageNum(std::string* err_message) {
    for (auto& path : record_files_) {
        cyber::record::RecordReader reader(path);
        if (!reader.IsValid()) {
            WRITE_ERR_MESSAGE(path + " is not a valid record file.");
        }
        total_message_num_ += reader.GetMessageNumber(conf_.reader_conf().point_cloud_channel());
    }
    AINFO << "Read total message num: " << total_message_num_;
    return true;
}

float ImagesCreator::GetProgress() {
    return image_renderer_->GetProgress();
};

}  // namespace tile_map_images_creator
}  // namespace apollo
