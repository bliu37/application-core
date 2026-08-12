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
#include <string>
#include <vector>
#include <cxxabi.h>
#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/map_creator/map_tool/utils/util.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"
#pragma once

namespace apollo {
namespace map_tool {

/**
 * @brief Get parking space polygon.
 *
 * @param editor_map The input editor map.
 * @param boundary_id the boundary id in editor map.
 *
 * @return The polygon of a parking space.
 */
apollo::hdmap::Polygon GetParkingPolygon(const apollo::map_tool::EditorMap &editor_map, const std::string &boundary_id);
}  // namespace map_tool
}  // namespace apollo
