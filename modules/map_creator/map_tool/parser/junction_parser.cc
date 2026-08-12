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
#include "modules/map_creator/map_tool/parser/junction_parser.h"

namespace apollo {
namespace map_tool {

apollo::hdmap::Polygon GetJunctionPolygon(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &boundary_id) {
    return GetPolygonByBoundaryId(editor_map, boundary_id);
}

bool GetJunctionType(const apollo::map_tool::Junction &junction, apollo::hdmap::Junction::Type &junction_type) {
    apollo::map_tool::JunctionAttribute::Type type = junction.attr().type();
    if (type == apollo::map_tool::JunctionAttribute::UNKNOWN) {
        junction_type = apollo::hdmap::Junction::UNKNOWN;
    } else if (type == apollo::map_tool::JunctionAttribute::IN_ROAD) {
        junction_type = apollo::hdmap::Junction::IN_ROAD;
    } else if (type == apollo::map_tool::JunctionAttribute::CROSS_ROAD) {
        junction_type = apollo::hdmap::Junction::CROSS_ROAD;
    } else if (type == apollo::map_tool::JunctionAttribute::FORK_ROAD) {
        junction_type = apollo::hdmap::Junction::FORK_ROAD;
    } else if (type == apollo::map_tool::JunctionAttribute::MAIN_SIDE) {
        junction_type = apollo::hdmap::Junction::MAIN_SIDE;
    } else if (type == apollo::map_tool::JunctionAttribute::DEAD_END) {
        junction_type = apollo::hdmap::Junction::DEAD_END;
    } else {
        AERROR << "Unknown junction type: " << type;
        return false;
    }
    return true;
}

}  // namespace map_tool
}  // namespace apollo
