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

#include "modules/map_creator/map_tool/parser/traffic_signal_parser.h"

#include <limits>
namespace apollo {
namespace map_tool {

#define TS_GH_SEARCH_RADIS 10
using apollo::common::math::Vec2d;
using apollo::hdmap::Signal_Type;
using apollo::map_tool::SubSignal_Type;
using apollo::map_tool::TrafficSignal_Type;
using Hdmap_Subsignal_Type = apollo::hdmap::Subsignal_Type;

std::string curr_base_map_dir = "";

apollo::hdmap::Signal::Type GetSignalType(const apollo::map_tool::TrafficSignal &signal) {
    if (signal.type() == TrafficSignal_Type::TrafficSignal_Type_MIX_2_HORIZONTAL) {
        return Signal_Type::Signal_Type_MIX_2_HORIZONTAL;
    }
    if (signal.type() == TrafficSignal_Type::TrafficSignal_Type_MIX_2_VERTICAL) {
        return Signal_Type::Signal_Type_MIX_2_VERTICAL;
    }
    if (signal.type() == TrafficSignal_Type::TrafficSignal_Type_MIX_3_HORIZONTAL) {
        return Signal_Type::Signal_Type_MIX_3_HORIZONTAL;
    }
    if (signal.type() == TrafficSignal_Type::TrafficSignal_Type_MIX_3_VERTICAL) {
        return Signal_Type::Signal_Type_MIX_3_VERTICAL;
    }
    if (signal.type() == TrafficSignal_Type::TrafficSignal_Type_SINGLE) {
        return Signal_Type::Signal_Type_SINGLE;
    }
    return Signal_Type::Signal_Type_UNKNOWN;
}

apollo::hdmap::Subsignal::Type GetSubSignalType(const apollo::map_tool::SubSignal &sub_signal) {
    if (sub_signal.type() == SubSignal_Type::SubSignal_Type_ARROW_FORWARD) {
        return Hdmap_Subsignal_Type::Subsignal_Type_ARROW_FORWARD;
    }
    if (sub_signal.type() == SubSignal_Type::SubSignal_Type_ARROW_LEFT) {
        return Hdmap_Subsignal_Type::Subsignal_Type_ARROW_LEFT;
    }
    if (sub_signal.type() == SubSignal_Type::SubSignal_Type_ARROW_RIGHT) {
        return Hdmap_Subsignal_Type::Subsignal_Type_ARROW_RIGHT;
    }
    if (sub_signal.type() == SubSignal_Type::SubSignal_Type_ARROW_U_TURN) {
        return Hdmap_Subsignal_Type::Subsignal_Type_ARROW_U_TURN;
    }
    if (sub_signal.type() == SubSignal_Type::SubSignal_Type_CIRCLE) {
        Hdmap_Subsignal_Type::Subsignal_Type_ARROW_LEFT_AND_FORWARD;
    }
    return Hdmap_Subsignal_Type::Subsignal_Type_UNKNOWN;
}

bool GetSignalID(const apollo::map_tool::EditorMap &editor_map, std::string &stop_line_id, std::string *signal_id) {
    auto iter = std::find_if(
            editor_map.traffic_signal().begin(), editor_map.traffic_signal().end(), [&](const auto &iter) {
                return iter.stop_line_id() == stop_line_id;
            });
    if (iter == editor_map.traffic_signal().end()) {
        AERROR << "Can't find signal id from stop line id: " << stop_line_id;
        return false;
    }
    *signal_id = apollo::map_tool::TrafficSignal(*iter).id();
    return true;
}

std::pair<apollo::common::math::Vec2d, apollo::common::math::Vec2d> CalculateSingleProjection(
        const double &heading,
        const Vec2d &point,
        double &length) {
    double start_x = point.x() - length * std::cos(heading);
    double start_y = point.y() - length * std::sin(heading);
    double end_x = point.x() + length * std::cos(heading);
    double end_y = point.y() + length * std::sin(heading);
    return {Vec2d(start_x, start_y), Vec2d(end_x, end_y)};
}

std::pair<double, double> GetSignalHeightLength(const TrafficSignal_Type &signal_type) {
    double height = 0;
    double length = 0;
    switch (signal_type) {
        // 水平两灯
    case TrafficSignal_Type::TrafficSignal_Type_MIX_2_HORIZONTAL:
        height = 0.45;
        length = 0.8;
        break;
    case TrafficSignal_Type::TrafficSignal_Type_MIX_2_VERTICAL:
        height = 0.8;
        length = 0.45;
        break;
    case TrafficSignal_Type::TrafficSignal_Type_MIX_3_HORIZONTAL:
        height = 0.45;
        length = 1.1;
        break;
    case TrafficSignal_Type::TrafficSignal_Type_MIX_3_VERTICAL:
        height = 1.1;
        length = 0.45;
        break;
    case TrafficSignal_Type::TrafficSignal_Type_SINGLE:
        height = 0.45;
        length = 0.45;
    default:
        break;
    }

    return {height, length};
}

apollo::hdmap::Polygon GetSignalPolygon(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::TrafficSignal &signal) {
    double ground_height = GetGroundHeight(
            signal.center().x() + editor_map.basemap_center().x(),
            signal.center().y() + editor_map.basemap_center().y(),
            TS_GH_SEARCH_RADIS);
    double signal_pole_height = ground_height + signal.height();
    std::pair<double, double> signal_height_length = GetSignalHeightLength(signal.type());
    std::pair<Vec2d, Vec2d> start_end_points = CalculateSingleProjection(
            signal.heading(), Vec2d(signal.center().x(), signal.center().y()), signal_height_length.second);
    apollo::hdmap::Polygon polygon;
    // Store points clockwise from the starting point.
    apollo::common::PointENU *point = polygon.add_point();
    point->set_x(start_end_points.first.x() + editor_map.basemap_center().x());
    point->set_y(start_end_points.first.y() + editor_map.basemap_center().y());
    point->set_z(signal_pole_height);
    point = polygon.add_point();
    point->set_x(start_end_points.first.x() + editor_map.basemap_center().x());
    point->set_y(start_end_points.first.y() + editor_map.basemap_center().y());
    point->set_z(signal_pole_height + signal_height_length.first);
    point = polygon.add_point();
    point->set_x(start_end_points.second.x() + editor_map.basemap_center().x());
    point->set_y(start_end_points.second.y() + editor_map.basemap_center().y());
    point->set_z(signal_pole_height + signal_height_length.first);
    point = polygon.add_point();
    point->set_x(start_end_points.second.x() + editor_map.basemap_center().x());
    point->set_y(start_end_points.second.y() + editor_map.basemap_center().y());
    point->set_z(signal_pole_height);
    return polygon;
}

}  // namespace map_tool
}  // namespace apollo