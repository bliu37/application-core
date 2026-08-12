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

#include <string>

#include <nlohmann/json.hpp>

#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/util.h"
#include "modules/common/util/json_util.h"
#include "modules/map_creator/map_tool/common/common.h"
#include "modules/map_creator/map_tool/parser/map_parser.h"
#include "modules/map_creator/map_tool/parser/traffic_signal_parser.h"
/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Load editor map
 *
 * @param filename: The editor map file path.
 * @param data: The output EditorMap protobuf object.
 *
 * @return A Status object.
 */
Status LoadEditorMap(const std::string& filename, EditorMap* data);

/**
 * @brief Load editor map
 *
 * @param filename: The editor map file path.
 * @param data: The output json data
 *
 * @return A Status object.
 */
Status LoadEditorMapToJson(const std::string& filename, nlohmann::json* data);

/**
 * @brief Save editor map
 *
 * @param filename: The output map file path.
 * @param data: The input EditorMap protobuf object.
 *
 * @return A Status object.
 */
Status SaveEditorMap(const std::string& filename, const EditorMap& data);

/**
 * @brief Save editor map
 *
 * @param filename: The output map file path.
 * @param data: The input json data.
 *
 * @return A Status object.
 */
Status SaveEditorMapFromJson(const std::string& filename, const nlohmann::json& data);

/**
 * @brief Convert editor map to apollo map from editor map file.
 *
 * @param filename: The editor map file path.
 * @param apollo_map: The ouput apollo map object.
 *
 * @return A Status object.
 */
Status ConvertToApolloMap(const std::string& filename, apollo::hdmap::Map* apollo_map);

/**
 * @brief Convert editor map to apollo map from json file
 *
 * @param data: The editor map json data
 * @param apollo_map: The ouput apollo map object.
 */
Status ConvertToApolloMap(const nlohmann::json& data, apollo::hdmap::Map* apollo_map);
/**
 * @brief Convert editor map to apollo map from EditorMap protobuf object.
 *
 * @param editor_map: The input EditorMap protobuf object.
 * @param apollo_map: The ouput apollo map object.
 *
 * @return A Status object.
 */
Status ConvertToApolloMap(EditorMap& editor_map, apollo::hdmap::Map* apollo_map);

/**
 * @brief Release apollo map, include base map, simulation map, and routing map.
 *
 * @param filename: The output apllo map data directory name.
 * @param data: The input apollo Map json.
 *
 * @return A Status object.
 */
Status ReleaseApolloMap(const std::string& filename, const nlohmann::json& data);

/**
 * @brief Release apollo map, include base map, simulation map, and routing map.
 *
 * @param filename: The output apllo map data directory name.
 * @param data: The input apollo Map object.
 *
 * @return A Status object.
 */
Status ReleaseApolloMap(const std::string& filename, apollo::hdmap::Map& apollo_map);

/**
 * @brief Quality inspection and verification before map release.
 *
 * @param data The input map json data.
 * @param error_message If there is a problem with the map, this is the specific error message after the map quality
 * inspection.
 *
 * @return True if the quality inspection is successful, false otherwise.
 */
bool MapValidityCheck(apollo::hdmap::Map& apollo_map, nlohmann::json* error_message);

}  // namespace map_tool
}  // namespace apollo
