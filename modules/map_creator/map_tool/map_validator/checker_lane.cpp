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
#include "checker_lane.h"

namespace apollo {
namespace map_tool {

bool CheckerLane::Check(std::vector<ErrorMessage>& err_messages) {
    std::shared_ptr<apollo::hdmap::Map> hdmap = this->hdmap_message_;

    if (hdmap->lane_size() == 0) {
        AERROR << "None lane is contained in hdmap";
    }

    for (int i = 0; i < hdmap->lane_size(); ++i) {
        const apollo::hdmap::Lane& lane = hdmap->lane(i);
        if (conf_.enable_width_check()) {
            if (!LaneWidthCheck(lane, err_messages)) {
                AERROR << "lane width check fail with lane id = " << lane.id().id();
                return false;
            }
        }
        if (conf_.enable_curvature_check()) {
            if (!LaneCurveCheck(lane, err_messages)) {
                AERROR << "lane curvature check fail with lane id = " << lane.id().id();
                return false;
            }
        }
        if (conf_.enable_lane_forward_check()) {
            if (!LaneForwardCheck(lane, err_messages)) {
                AERROR << "lane forward check fail with lane id = " << lane.id().id();
                return false;
            }
        }
    }
    return true;
}

bool CheckerLane::LaneWidthCheck(const apollo::hdmap::Lane& lane, std::vector<ErrorMessage>& err_messages) {
    if (lane.left_sample_size() != lane.right_sample_size()) {
        AERROR << "sample size of lane id = " << lane.id().id() << " not match, left = " << lane.left_sample_size()
               << " right = " << lane.right_sample_size();
        return false;
    }

    for (int i = 0; i < lane.left_sample_size(); ++i) {
        apollo::hdmap::LaneSampleAssociation left_sample, right_sample;
        left_sample = lane.left_sample(i);
        right_sample = lane.right_sample(i);

        if (!left_sample.has_s() || !left_sample.has_width() || !right_sample.has_s() || !right_sample.has_width()) {
            AERROR << "sample point lack of s value of width value, lane id = " << lane.id().id();
            continue;
        }

        if (std::fabs(left_sample.s() - right_sample.s()) > 1e-6) {
            AERROR << boost::format(
                              "sample point of lane id = %s with s value error, left_sample s = %.7f, right_sample s = "
                              "%.7f")
                            % lane.id().id() % left_sample.s() % right_sample.s();
            continue;
        }

        double width = left_sample.width() + right_sample.width();

        ErrorMessage message;
        if (width < conf_.width_warning_lower_bound()) {
            AERROR << boost::format("lane id = %s width too narrow (warn), %.6f + %.6f = %.6f < %.6f ") % lane.id().id()
                            % left_sample.width() % right_sample.width() % width % conf_.width_warning_lower_bound();
            RETURN_VAL_IF(!AddNewErrorMessage(err_messages, LaneTooNarrowWarn, lane.id().id()), false);
            break;
        } else if (width > conf_.width_warning_upper_bound()) {
            AERROR << boost::format("lane id = %s width too wide (warn), %.6f + %.6f = %.6f > %.6f ") % lane.id().id()
                            % left_sample.width() % right_sample.width() % width % conf_.width_warning_upper_bound();
            RETURN_VAL_IF(!AddNewErrorMessage(err_messages, LaneTooWideWarn, lane.id().id()), false);
            break;
        }
    }
    return true;
}

bool CheckerLane::LaneCurveCheck(const apollo::hdmap::Lane& lane, std::vector<ErrorMessage>& err_messages) {
    auto p_lane = hdmap_instance_->GetLaneById(lane.id());
    for (double key_accumulate_s : p_lane->accumulate_s()) {
        double key_point_curvature = p_lane->Curvature(key_accumulate_s);

        if (conf_.has_curvature_upper_bound() && fabs(key_point_curvature) > conf_.curvature_upper_bound()) {
            AERROR << boost::format("lane id = %s curvature too large (warn), %.6f > %.6f ") % lane.id().id()
                            % key_point_curvature % conf_.curvature_upper_bound();
            RETURN_VAL_IF(!AddNewErrorMessage(err_messages, LaneCurvatureTooLargeWarn, lane.id().id()), false);
            break;
        }
    }
    return true;
}

bool CheckerLane::LaneForwardCheck(const apollo::hdmap::Lane& lane, std::vector<ErrorMessage>& err_messages) {
    const auto& central_curve = lane.central_curve();

    auto p_lane = hdmap_instance_->GetLaneById(lane.id());
    const std::vector<double>& headings = p_lane->headings();

    double max_heading_difference = 0.0;
    for (int i = 0; i < headings.size() - 1; ++i) {
        double heading_now = headings[i];
        double heading_next = headings[i + 1];
        double relative_heading = heading_next - heading_now;
        double heading_difference_1 = std::fmod(relative_heading + M_PI * 2, M_PI * 2);
        double heading_difference_2 = std::fmod(-relative_heading + M_PI * 2, M_PI * 2);

        double heading_difference = std::min(heading_difference_1, heading_difference_2);
        max_heading_difference = std::max(max_heading_difference, heading_difference);
    }

    ADEBUG << boost::format("Land id = %s max heading difference = %.6f") % lane.id().id() % max_heading_difference;
    if (max_heading_difference > lane_forward_check_threshold_) {
        AERROR << boost::format("Lane id = %s heading difference too large, difference = %.6f") % lane.id().id()
                        % max_heading_difference;
        RETURN_VAL_IF(!AddNewErrorMessage(err_messages, LaneCentralCurveNotForwardError, lane.id().id()), false);
    }

    return true;
}

}  // namespace map_tool
}  // namespace apollo
