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

#include "modules/common_msgs/map_msgs/map_signal.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/map_creator/map_tool/utils/util.h"

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Get signal type.
 *
 * @param signal The input editor map traffic signal.
 *
 * @return The output hdmap signal type.
 */
apollo::hdmap::Signal::Type GetSignalType(const apollo::map_tool::TrafficSignal &signal);

/**
 * @brief Get sub signal type.
 *
 * @param sub_signal The input sub signal editor map traffic sub signal type.
 *
 * @return The output hdmap sub signal type.
 */
apollo::hdmap::Subsignal::Type GetSubSignalType(const apollo::map_tool::SubSignal &sub_signal);

/**
 * @brief Get traffic signal id by stop line id.
 *
 * @param editor_map The input editor map.
 * @param stop_line_id The input stop line id.
 * @param signal_id The output signal id.
 */
bool GetSignalID(const apollo::map_tool::EditorMap &editor_map, std::string &stop_line_id, std::string *signal_id);

/**
 * @brief Get the border range of the signal light.
 *
 * @param editor_map The input editor map.
 * @param signal The input signal.
 *
 * @return The polygon of input signal.
 */
apollo::hdmap::Polygon GetSignalPolygon(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::TrafficSignal &signal);

/**
 * @brief Obtain the length and height of the signal light through the unified specifications of the signal light.
 *
 * @param signal_type The input signal type.
 *
 * @return A pair with signal height and length, first is signal height, second is signal width.
 */
std::pair<double, double> GetSignalHeightLength(const apollo::map_tool::TrafficSignal_Type &signal_type);

/**
 * @brief According to the heading and the center point of the x-y plane of the signal light, calculate the projection
 * of the signal light on the x-y plane.
 */
std::pair<apollo::common::math::Vec2d, apollo::common::math::Vec2d> CalculateSingleProjection(
        const double &heading,
        const apollo::common::math::Vec2d &point,
        double &length);

// /**
//  * @brief Set the current base map path.
//  *
//  * @param base_map_dir The input base map dir.
//  */
// void SetCurrBaseMapDir(const std::string &base_map_dir);

}  // namespace map_tool
}  // namespace apollo