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
#include <vector>

#include <nlohmann/json.hpp>

#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/common_msgs/map_msgs/map_junction.pb.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"
#include "modules/routing/proto/routing_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/util.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/points_downsampler.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map_creator/map_tool/common/common.h"
#include "modules/map_creator/map_tool/common/map_tool_gflags.h"
#include "modules/map_creator/map_tool/map_validator/map_validator.h"
#include "modules/map_creator/map_tool/parser/barrier_gate_parser.h"
#include "modules/map_creator/map_tool/parser/crosswalk_parser.h"
#include "modules/map_creator/map_tool/parser/junction_parser.h"
#include "modules/map_creator/map_tool/parser/lane_parser.h"
#include "modules/map_creator/map_tool/parser/overlap_parser.h"
#include "modules/map_creator/map_tool/parser/parking_space_parser.h"
#include "modules/map_creator/map_tool/parser/road_parser.h"
#include "modules/map_creator/map_tool/parser/speed_bump_parser.h"
#include "modules/map_creator/map_tool/parser/traffic_signal_parser.h"
#include "modules/routing/topo_creator/graph_creator.h"

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Get hdmap lane from editor map.
 *
 * @param editor_map The editor map.
 * @param lane_relation_parser The relation between editor map and hdmap.
 * @param map The output hdmap with lane information.
 *
 * @return true if success, otherwise false.
 */
bool CovertLanes(
        const apollo::map_tool::EditorMap &editor_map,
        const std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        apollo::hdmap::Map *map);

/**
 * @brief Get hdmap lane from editor map.
 *
 * @param editor_map The editor map.
 * @param map The output hdmap with junction information.
 *
 * @return true if success, otherwise false.
 */
bool CovertJunctions(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get hdmap speed bump from editor map.
 *
 * @param editor_map The editor map.
 * @param map The output hdmap with speed bump information.
 *
 * @return true if success, otherwise false.
 */
bool CovertSpeedBumps(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get hdmap crosswalk from editor map.
 *
 * @param editor_map The editor map.
 * @param map The output hdmap with crosswalk information.
 *
 * @return true if success, otherwise false.
 */
bool CovertCrosswalks(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get hdmap parking space from editor map.
 *
 * @param editor_map The editor map.
 * @param map The output hdmap with parking space information.
 */
bool CovertParkingSpaces(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get traffic signal from editor map
 */
bool CovertTrafficSignals(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get yield sign from editor map
 */
bool CovertYieldSigns(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get stop sign from editor map
 */
bool CovertStopSigns(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get area from editor map
 */
bool CovertArea(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get barrier gate from editor map
 */
bool CovertBarrierGate(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get map header from editor map
 *
 * @param editor_map The editor map.
 * @param map The output hdmap with header information.
 */
bool CovertHeader(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Get hdmap from editor map.
 *
 * @param json The editor map in json format.
 * @param map The output hdmap.
 *
 * @return true if success, otherwise false.
 */
bool EditorMapParser(const nlohmann::json &json, apollo::hdmap::Map *map);

/**
 * @brief Get hdmap from editor map.
 *
 * @param json The editor map in proto format.
 * @param map The output hdmap.
 *
 * @return true if success, otherwise false.
 */
bool EditorMapParser(apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map);

/**
 * @brief Generate simulation map from editor map.
 *
 * @param file_name File name for generating simulation map.
 * @param map The input hdmap.
 */
void GenerateSimMap(const std::string file_name, apollo::hdmap::Map *map);

/**
 * @brief Genarate routing map from editor map.
 *
 * @param base_map_path The path of the base map.
 *
 * @return true if success, otherwise false.
 *
 */
bool GenerateRoutingMap(const std::string &base_map_path);

/**
 * @brief Downsample hdmap boundary curve for generate simulation map.
 *
 * @param curve The input hdmap boundary curve.
 */
void DownsampleCurve(apollo::hdmap::Curve *curve);

/**
 * @brief Save simulation map to txt file and binary file.
 *
 * @param file_name The name of the output file.
 * @param map_pb The input hdmap.
 */
void SaveSimMap(const std::string file_name, const apollo::hdmap::Map &map_pb);

/**
 * @brief Add reverse lane to input editor map.
 *
 * @param editor_map The input eidtor map.
 */
void AddReverseLaneToEditorMap(apollo::map_tool::EditorMap *editor_map);

/**
 * @brief Convert curve to lane.
 *
 * @param editor_map The input editor map.
 * @param lane_relation_parser relation between editor map and hdmap.
 */
void ConvertCurveTolane(
        apollo::map_tool::EditorMap *editor_map,
        std::map<std::string, apollo::map_tool::LaneParser *> *lane_relation_parser);

/**
 * @brief Verify the validity of the map.
 *
 * @param map The input HDmap.
 * @param result The Validation results.
 *
 * @return Return true if map validity verification is completed successfully. otherwise false.
 */
bool VerifyMapValidity(apollo::hdmap::Map &map, ValidatorResult *result);

/**
 * @brief Open geographic information file.
 *
 * @param base_map_dir Basemap directory being opened.
 * @param geo_file The geographic information json file.
 *
 * @return Return true if geo_file exists, otherwise false.
 */
bool OpenGeoInfoFile(const std::string &base_map_dir, nlohmann::json *geo_file);

}  // namespace map_tool
}  // namespace apollo
