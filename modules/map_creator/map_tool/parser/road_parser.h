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

#include <list>

#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "cyber/common/log.h"
#include "modules/map_creator/map_tool/parser/lane_parser.h"
#include "modules/map_creator/map_tool/utils/util.h"
/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Add road into map.
 *
 * @param hdmap The hdmap that needs to add overlap information.
 *
 * @return True if success.
 */
bool AddRoads(apollo::hdmap::Map* hdmap, apollo::map_tool::EditorMap& editor_map);

/**
 * @brief Find rightmost lane of a road.
 *
 * @param lane_map Lane id and lane mapping map.
 * @param lane The start lane of a road.
 * @param mapped_lane Set to determine whether the lane has been traversed.
 * @param ids The right lane ids of the road.
 *
 * @return The rightmost lane of a road.
 */
const apollo::hdmap::Lane* FindRightmostLane(
        std::map<std::string, apollo::hdmap::Lane>& lane_map,
        const apollo::hdmap::Lane& lane,
        std::set<std::string>& mapped_lane,
        std::list<apollo::hdmap::Id>* ids);

/**
 * @brief Find leftmost lane of a road.
 *
 * @param lane_map Lane id and lane mapping map.
 * @param lane The start lane of a road.
 * @param mapped_lane Set to determine whether the lane has been traversed.
 * @param ids The left lane ids of the road.
 *
 * @return The leftmost lane of a road.
 */
const apollo::hdmap::Lane* FindLeftmostLane(
        std::map<std::string, apollo::hdmap::Lane>& lane_map,
        const apollo::hdmap::Lane& lane,
        std::set<std::string>& mapped_lane,
        std::list<apollo::hdmap::Id>* ids);

/**
 * @brief Generate a road from a lane.
 *
 * @param ids Lane ids of the road.
 * @param rightmost_lane The rightmost lane of the road.
 * @param leftmost_lane The leftmost lane of the road.
 * @param road_id The road id.
 * @param road The output road.
 */
void GenerateRoad(
        std::list<apollo::hdmap::Id>& ids,
        const apollo::hdmap::BoundaryEdge& left_road_edge,
        const apollo::hdmap::BoundaryEdge& right_road_edge,
        const apollo::hdmap::Id& road_id,
        apollo::hdmap::Road* road);

/**
 * @brief Get a road boundary points.
 *
 * @param point_map editor point map.
 * @param road_boundary The output road boundary.
 * @param if_curve If boundary is curve.
 */
std::vector<apollo::common::math::Vec2d> GetRoadBoundary(
        std::map<std::string, apollo::map_tool::Point>& point_map,
        const apollo::map_tool::RoadBoundary& road_boundary,
        bool if_curve);

/**
 * @brief Add road sample to Hdmap Lane.
 *
 * @param left_road_boundary The left road boundary.
 * @param right_road_boundary The right road boundary.
 * @param lane The hdmap lane.
 * @param map_offset The map_offset of hdmap and editor map.
 */
void CalculateRoadSample(
        apollo::hdmap::Map* map,
        std::list<apollo::hdmap::Id>& lane_ids,
        std::vector<apollo::common::math::Vec2d>& left_road_boundary,
        std::vector<apollo::common::math::Vec2d>& right_road_boundary,
        apollo::common::math::Vec2d& map_offset);

}  // namespace map_tool
}  // namespace apollo
