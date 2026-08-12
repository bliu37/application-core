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
#include <utility>
#include <vector>

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/map_msgs/map_junction.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "cyber/common/file.h"
#include "modules/map_creator/map_tool/utils/util.h"

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Get the junction polygon from the editor map.
 *
 * @param editor_map The editor map.
 * @param boundary_id The boundary id.
 *
 * @return The output polygon.
 */
apollo::hdmap::Polygon GetJunctionPolygon(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &boundary_id);

/**
 * @brief Get the junction type from the editor map.
 *
 * @param junction The query junction.
 * @param junction_type The output junction type.
 *
 * @return true on success, otherwise false.
 */
bool GetJunctionType(const apollo::map_tool::Junction &junction, apollo::hdmap::Junction::Type &junction_type);

}  // namespace map_tool
}  // namespace apollo
