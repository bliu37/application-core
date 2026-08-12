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
#include "checker_parking_space_size.h"
namespace apollo {
namespace map_tool {

inline bool CheckerParkingSpaceSize::DoubleEqual(double x1, double x2, double eps) {
    return std::fabs(x1 - x2) < eps;
}

inline double CheckerParkingSpaceSize::RelativeAngleCos(const common::math::Vec2d& l1, const common::math::Vec2d& l2) {
    return std::fabs(l1.InnerProd(l2)) / l1.Length() / l2.Length();
}

inline bool CheckerParkingSpaceSize::ZeroVectorCheck(const std::vector<common::math::Vec2d>& lines) {
    return std::none_of(lines.begin(), lines.end(), [this](const auto& line) {
        return DoubleEqual(line.Length(), 0, rectangle_length_judgement_eps_);
    });
}

inline bool CheckerParkingSpaceSize::RectangleCheck(const std::vector<common::math::Vec2d>& lines) {
    return DoubleEqual(lines[0].Length(), lines[2].Length(), rectangle_length_judgement_eps_)
            && DoubleEqual(lines[1].Length(), lines[3].Length(), rectangle_length_judgement_eps_)
            && DoubleEqual(RelativeAngleCos(lines[0], lines[1]), 0, rectangle_cos_angle_judgement_eps_)
            && DoubleEqual(RelativeAngleCos(lines[1], lines[2]), 0, rectangle_cos_angle_judgement_eps_);
}

bool CheckerParkingSpaceSize::Check(std::vector<ErrorMessage>& err_messages) {
    for (const hdmap::ParkingSpace& parking_space : hdmap_message_->parking_space()) {
        if (parking_space.has_polygon() && parking_space.polygon().point_size() == 4) {
            const auto& parking_space_polygon = parking_space.polygon();
            std::vector<common::math::Vec2d> points(4), lines(4);
            for (int i = 0; i < 4; ++i) {
                points[i] = common::math::Vec2d(parking_space_polygon.point(i).x(), parking_space_polygon.point(i).y());
            }
            for (int i = 0; i < 4; ++i) {
                lines[i] = points[(i + 1) % 4] - points[i];
            }

            if (ZeroVectorCheck(lines) && RectangleCheck(lines)) {
                double length = lines[0].Length(), width = lines[1].Length();

                if (length < width) {
                    std::swap(length, width);
                }
                AINFO << "parking space id = " << parking_space.id().id() << " is a rectangle, with length = " << length
                      << " width = " << width;

                if (conf_.has_length_error_lower_bound() && length < conf_.length_error_lower_bound()) {
                    AERROR << "parking space = " << parking_space.id().id() << " length too small";
                    RETURN_VAL_IF(
                            !AddNewErrorMessage(err_messages, ParkingSpaceLengthTooSmallError, parking_space.id().id()),
                            false);
                }

                if (conf_.has_width_error_lower_bound() && width < conf_.width_error_lower_bound()) {
                    AERROR << "parking space = " << parking_space.id().id() << " width too small";
                    RETURN_VAL_IF(
                            !AddNewErrorMessage(err_messages, ParkingSpaceWidthTooSmallError, parking_space.id().id()),
                            false);
                }

                if (conf_.has_length_warning_upper_bound() && length > conf_.length_warning_upper_bound()) {
                    AERROR << "parking space = " << parking_space.id().id() << " length too large";
                    RETURN_VAL_IF(
                            !AddNewErrorMessage(
                                    err_messages, ParkingSpaceLengthTooLargeWarning, parking_space.id().id()),
                            false);
                }

                if (conf_.has_width_warning_upper_bound() && width > conf_.width_warning_upper_bound()) {
                    AERROR << "parking space = " << parking_space.id().id() << " width too large";
                    RETURN_VAL_IF(
                            !AddNewErrorMessage(
                                    err_messages, ParkingSpaceWidthTooLargeWarning, parking_space.id().id()),
                            false);
                }

            } else {
                AERROR << "parking space id = " << parking_space.id().id()
                       << " isn't a rectangle, detail = " << parking_space.DebugString();
                AERROR << "l0 = " << lines[0].Length() << " l1 = " << lines[1].Length() << " l2 = " << lines[2].Length()
                       << " l3 = " << lines[3].Length();
                AERROR << "cos<l0, l1> = " << lines[0].InnerProd(lines[1]) / lines[0].Length() / lines[1].Length()
                       << ", cos<l1, l2>" << lines[1].InnerProd(lines[2]) / lines[1].Length() / lines[2].Length();

                AERROR << DoubleEqual(lines[0].Length(), lines[2].Length(), rectangle_length_judgement_eps_)
                       << DoubleEqual(lines[1].Length(), lines[3].Length(), rectangle_length_judgement_eps_)
                       << DoubleEqual(RelativeAngleCos(lines[0], lines[1]), 0, rectangle_cos_angle_judgement_eps_)
                       << DoubleEqual(RelativeAngleCos(lines[1], lines[2]), 0, rectangle_cos_angle_judgement_eps_);

                RETURN_VAL_IF(
                        !AddNewErrorMessage(err_messages, ParkingSpaceNotRectangle, parking_space.id().id()), false);
            }

        } else {
            AERROR << "parking space id = " << parking_space.id().id()
                   << " has a wrong polygon, detail = " << parking_space.DebugString();
        }
    }
    return true;
}

}  // namespace map_tool
}  // namespace apollo
