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
#include "modules/map_creator/map_tool/parser/parking_space_parser.h"

#include <string>
#include <vector>

#pragma once

namespace apollo {
namespace map_tool {

apollo::hdmap::Polygon GetParkingPolygon(
        const apollo::map_tool::EditorMap& editor_map,
        const std::string& boundary_id) {
    apollo::hdmap::Polygon polygon;
    std::vector<std::string> point_ids;
    double map_offset_x = editor_map.basemap_center().x();
    double map_offset_y = editor_map.basemap_center().y();
    GetPointIdByBoundaryId(editor_map, boundary_id, point_ids);
    point_ids.pop_back();
    std::map<std::string, apollo::map_tool::Point> point_map;
    for (const auto& point : editor_map.point()) {
        point_map[point.id()] = point;
    }
    for (const auto& point_id : point_ids) {
        apollo::common::PointENU* point = polygon.add_point();
        point->set_x(point_map[point_id].position().x() + map_offset_x);
        point->set_y(point_map[point_id].position().y() + map_offset_y);
        point->set_z(point_map[point_id].position().z());
    }
    return polygon;
}

}  // namespace map_tool
}  // namespace apollo
