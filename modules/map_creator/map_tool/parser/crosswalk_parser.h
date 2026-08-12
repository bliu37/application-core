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
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "modules/map_creator/map_tool/utils/util.h"

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Get crosswalk polygon from editor_map.
 *
 * @param editor_map The editor_map.
 * @param boundary_id The id of the boundary id of editor map.
 *
 * @return The polygon of crosswalk.
 */
apollo::hdmap::Polygon GetCrosswalkPolygon(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &boundary_id);

}  // namespace map_tool
}  // namespace apollo
