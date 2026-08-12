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
#include "modules/map_creator/map_tool/parser/speed_bump_parser.h"

namespace apollo {
namespace map_tool {
using apollo::common::math::Vec2d;

void GetSpeedBumpCurve(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::SpeedBump &speed_bump,
        const std::string &boundary_id,
        apollo::hdmap::Curve *curve) {
    std::vector<std::string> point_ids;
    std::vector<Vec2d> points;
    GetPointIdByBoundaryId(editor_map, boundary_id, point_ids);
    std::unordered_map<std::string, apollo::map_tool::Point> point_map;
    for (const auto &point : editor_map.point()) {
        point_map[point.id()] = point;
    }
    for (const auto &point_id : point_ids) {
        points.push_back(Vec2d(point_map[point_id].position().x(), point_map[point_id].position().y()));
    }
    Vec2d map_offset(editor_map.basemap_center().x(), editor_map.basemap_center().y());
    GetBoundaryCurve(points, curve, map_offset);
}

}  // namespace map_tool
}  // namespace apollo
