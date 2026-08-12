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
#include "map_validator.h"
#include "boost/multi_array.hpp"

namespace apollo {
namespace map_tool {

MapValidator::MapValidator() {
    hdmap_message_ = std::make_shared<apollo::hdmap::Map>();
    hdmap_instance_ = std::make_shared<apollo::hdmap::HDMap>();
}

bool MapValidator::Init(
        const std::string& checker_conf_path,
        const std::string& errcode_conf_path,
        const std::string& map_bin_path) {
    apollo::hdmap::Map map_instance;
    if (!apollo::cyber::common::GetProtoFromBinaryFile(map_bin_path, &map_instance)) {
        AERROR << "load hdmap proto error, path = " << map_bin_path;
        return false;
    }
    AINFO << "load hdmap bin file : " << map_bin_path;

    return Init(checker_conf_path, errcode_conf_path, map_instance);
}

bool MapValidator::Init(
        const std::string& checker_conf_path,
        const std::string& errcode_conf_path,
        const apollo::hdmap::Map& map) {
    if (!apollo::cyber::common::GetProtoFromASCIIFile(checker_conf_path, &config_)) {
        AERROR << "load checker conf error, path = " << checker_conf_path;
        return false;
    }
    AINFO << "load config file : " << checker_conf_path;

    if (!ErrcodeFactory::Instance()->Init(errcode_conf_path)) {
        AERROR << "errcode factory init error, with path = " << errcode_conf_path;
        return false;
    }

    if (hdmap_instance_->LoadMapFromProto(map) != cyber::SUCC) {
        AERROR << "hdmap instance init not success";
        return false;
    }

    hdmap_message_ = std::make_shared<apollo::hdmap::Map>(map);

    checker_lane_ = std::make_unique<CheckerLane>(hdmap_message_, hdmap_instance_);
    if (!checker_lane_->Init(config_.check_lane_config())) {
        AERROR << "lane checker Init error";
        return false;
    }

    checker_overlap_ = std::make_unique<CheckerOverlap>(hdmap_message_, hdmap_instance_);
    if (!checker_overlap_->Init(config_.check_overlap_config())) {
        AERROR << "lane checker Init error";
        return false;
    }

    checker_parking_space_size_ = std::make_unique<CheckerParkingSpaceSize>(hdmap_message_, hdmap_instance_);
    if (!checker_parking_space_size_->Init(config_.check_parking_space_size_config())) {
        AERROR << "lane checker Init error";
        return false;
    }

    return true;
}

bool MapValidator::Check(ValidatorResult& result) {
    std::vector<ErrorMessage> error_messages;
    if (config_.has_check_lane_config()) {
        if (!checker_lane_->Check(error_messages)) {
            AERROR << "check lane fail";
            return false;
        }
    }

    if (config_.has_check_overlap_config()) {
        if (!checker_overlap_->Check(error_messages)) {
            AERROR << "check overlap fail";
            return false;
        }
    }

    if (config_.has_check_overlap_config()) {
        if (!checker_parking_space_size_->Check(error_messages)) {
            AERROR << "check parking space size fail";
            return false;
        }
    }

    for (const ErrorMessage& message : error_messages) {
        result.add_error_message()->CopyFrom(message);
    }
    return true;
}

}  // namespace map_tool
}  // namespace apollo