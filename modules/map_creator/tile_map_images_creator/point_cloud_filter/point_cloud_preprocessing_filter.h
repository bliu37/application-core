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

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"
#include "modules/map_creator/tile_map_images_creator/common/frame.h"

namespace apollo {
namespace tile_map_images_creator {

class PointCloudPreprocessingFilter {
public:
    void Filter(std::vector<PointXYZIT_D>& points);

    void Init(const FilterConf& filter_conf);

    void UpdatePosition(const std::shared_ptr<apollo::common::PointENU>& position) {
        position_ = position;
    }

private:
    bool HeightJudgement(const PointXYZIT_D& point);

    bool DistanceJudgement(const PointXYZIT_D& point);

    std::shared_ptr<apollo::common::PointENU> position_;

    bool enable_height_filter_ = false;
    double lower_height_limit_relative_to_pose_ = 0.0, upper_height_limit_relative_to_pose_ = 0.0;

    bool enable_distance_filter_ = false;
    double upper_distance_limit_ = 0.0, lower_distance_limit_ = 0.0;
};

}  // namespace tile_map_images_creator
}  // namespace apollo