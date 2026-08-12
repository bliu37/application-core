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

class CheckerParkingSpaceSize final : public CheckerBase {
public:
    CheckerParkingSpaceSize() = default;

    CheckerParkingSpaceSize(
            const std::shared_ptr<apollo::hdmap::Map>& hdmap_message,
            const std::shared_ptr<apollo::hdmap::HDMap>& hdmap_instance) :
            CheckerBase(hdmap_message, hdmap_instance) {}

    bool Init(const CheckerParkingSpaceSizeConfig& config) {
        conf_ = config;
        return true;
    }

    bool Check(std::vector<ErrorMessage>& err_messages);

private:
    inline bool DoubleEqual(double x1, double x2, double eps = 1e-2);

    inline double RelativeAngleCos(const common::math::Vec2d& l1, const common::math::Vec2d& l2);

    inline bool ZeroVectorCheck(const std::vector<common::math::Vec2d>& lines);

    inline bool RectangleCheck(const std::vector<common::math::Vec2d>& lines);

    CheckerParkingSpaceSizeConfig conf_;

    const double rectangle_length_judgement_eps_ = 0.05;
    const double rectangle_cos_angle_judgement_eps_ = 0.02;
};

}  // namespace map_tool
}  // namespace apollo
