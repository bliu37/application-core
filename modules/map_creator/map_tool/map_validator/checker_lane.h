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

#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/map_creator/map_tool/proto/map_validator.pb.h"
#include "checker_base.h"
#include "cyber/cyber.h"
#include "boost/format.hpp"
#include <vector>

namespace apollo {
namespace map_tool {

class CheckerLane : public CheckerBase {
public:
    CheckerLane() = default;

    CheckerLane(
            const std::shared_ptr<apollo::hdmap::Map>& hdmap_message,
            const std::shared_ptr<apollo::hdmap::HDMap>& hdmap_instance) :
            CheckerBase(hdmap_message, hdmap_instance) {}

    bool Init(const CheckerLaneConfig& config) {
        conf_ = config;
        lane_forward_check_threshold_ = conf_.maximum_angle_difference() / 180.0 * M_PI;
        return true;
    }

    bool Check(std::vector<ErrorMessage>& err_messages);

    bool LaneWidthCheck(const apollo::hdmap::Lane& lane, std::vector<ErrorMessage>& err_messages);

    bool LaneCurveCheck(const apollo::hdmap::Lane& lane, std::vector<ErrorMessage>& err_messages);

    bool LaneForwardCheck(const apollo::hdmap::Lane& lane, std::vector<ErrorMessage>& err_messages);

    CheckerLaneConfig conf_;
    double lane_forward_check_threshold_ = M_PI_2;
};

}  // namespace map_tool
}  // namespace apollo
