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
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/map_msgs/map_speed_bump.pb.h"

#include "modules/map_creator/map_tool/parser/lane_parser.h"
#pragma once

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Get speed bump curve from editor map.
 *
 * @param editor_map The editor map.
 * @param speed_bump The speed bump.
 * @param boundary_id The id of the boundary id.
 * @param curve The speed bump curve.
 */
void GetSpeedBumpCurve(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::SpeedBump &speed_bump,
        const std::string &boundary_id,
        apollo::hdmap::Curve *curve);

}  // namespace map_tool
}  // namespace apollo
