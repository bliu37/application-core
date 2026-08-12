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

#include "checker_base.h"
#include "modules/map_creator/map_tool/proto/map_validator.pb.h"

namespace apollo {
namespace map_tool {

class CheckerOverlap final : public CheckerBase {
public:
    CheckerOverlap() = default;

    CheckerOverlap(
            const std::shared_ptr<apollo::hdmap::Map>& hdmap_message,
            const std::shared_ptr<apollo::hdmap::HDMap>& hdmap_instance) :
            CheckerBase(hdmap_message, hdmap_instance) {}

    bool Init(const CheckerOverlapConfig& config) {
        conf_ = config;
        return true;
    }

    bool Check(std::vector<ErrorMessage>& err_messages);

private:
    CheckerOverlapConfig conf_;
    std::set<std::string> lane_id_set_;
    // 存储能关联lane的overlap
    std::set<std::string> overlap_lane_exist_mask_;
};

}  // namespace map_tool
}  // namespace apollo
