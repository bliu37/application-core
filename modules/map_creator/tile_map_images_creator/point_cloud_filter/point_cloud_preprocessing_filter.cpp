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
#include "point_cloud_preprocessing_filter.h"
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"
#include "cyber/cyber.h"
#include <boost/format.hpp>

namespace apollo {
namespace tile_map_images_creator {

void PointCloudPreprocessingFilter::Init(const FilterConf& filter_conf) {
    if (filter_conf.has_enable_height_filter() && filter_conf.enable_height_filter()) {
        enable_height_filter_ = true;
        lower_height_limit_relative_to_pose_ = filter_conf.lower_height_limit_relative_to_pose();
        upper_height_limit_relative_to_pose_ = filter_conf.upper_height_limit_relative_to_pose();
        AINFO << boost::format("enable height filter relative to pose, range = [%.3f, %.3f]")
                        % lower_height_limit_relative_to_pose_ % upper_height_limit_relative_to_pose_;
    }

    if (filter_conf.has_enable_distance_filter() && filter_conf.enable_distance_filter()) {
        enable_distance_filter_ = true;
        upper_distance_limit_ = filter_conf.upper_distance_limit();
        lower_distance_limit_ = filter_conf.lower_distance_limit();
        AINFO << boost::format("enable distance filter relative to pose, range = [%.3f, %.3f]") % lower_distance_limit_
                        % upper_distance_limit_;
    }
}

void PointCloudPreprocessingFilter::Filter(std::vector<PointXYZIT_D>& points) {
    std::vector<PointXYZIT_D> filter_points;

    int height_filter_pass_count = 0, distance_filter_pass_count = 0;

    for (const PointXYZIT_D& point : points) {
        bool height_judgement_result = !enable_height_filter_ | HeightJudgement(point);
        height_filter_pass_count += height_judgement_result;

        bool distance_judgement_result = !enable_distance_filter_ | DistanceJudgement(point);
        distance_filter_pass_count += distance_judgement_result;

        bool result = height_judgement_result & distance_judgement_result;
        if (result) {
            filter_points.push_back(point);
        }
    }

    AINFO << boost::format(
                     "raw point size = %d, after filter size = %d, height filter pass = %d, distance filter pass = %d")
                    % points.size() % filter_points.size() % height_filter_pass_count % distance_filter_pass_count;
    points.swap(filter_points);
}

bool PointCloudPreprocessingFilter::HeightJudgement(const PointXYZIT_D& point) {
    return (point.z >= position_->z() + lower_height_limit_relative_to_pose_
            && point.z <= position_->z() + upper_height_limit_relative_to_pose_);
}

bool PointCloudPreprocessingFilter::DistanceJudgement(const PointXYZIT_D& point) {
    double distance = hypot(point.x - position_->x(), point.y - position_->y());
    return (distance >= lower_distance_limit_ && distance <= upper_distance_limit_);
}

}  // namespace tile_map_images_creator
}  // namespace apollo
