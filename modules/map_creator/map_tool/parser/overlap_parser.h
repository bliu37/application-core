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

#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/common_msgs/map_msgs/map_overlap.pb.h"
#include "modules/common_msgs/map_msgs/map_speed_bump.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map_creator/map_tool/common/map_tool_gflags.h"
#include "modules/map_creator/map_tool/parser/lane_parser.h"
#include "modules/map_creator/map_tool/parser/traffic_signal_parser.h"
#include "modules/map_creator/map_tool/utils/util.h"

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {
/**
 * @brief Calculate all overlaps of hdmap.
 *
 * @param editor_map The input editor map.
 * @param hdmap The hdmap that needs to add overlap information.
 * @param lane_relation_parser The lane parser.
 *
 * @return Return true if calculate signal overlap success, else return false.
 */
bool CalculateOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser);

/**
 * @brief Calculate overlaps between lane and speed bump.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and speed bump overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate signal overlap success, else return false.
 */
bool CalculateLaneAndSpeedBumpOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and crosswalk.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and crosswalk overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate signal overlap success, else return false.
 */
bool CalculateLaneAndCrosswalkOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and signal.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and signal overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate signal overlap success, else return false.
 */
bool CalculateLaneAndSignalOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and signal.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and signal overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate signal overlap success, else return false.
 */
bool CalculateLaneAndJunctionOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and parking space.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and parking space overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate overlap success, else return false.
 */
bool CalculateLaneAndParkingSpaceOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and yield sign.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and parking space overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate overlap success, else return false.
 */
bool CalculateYieldSignAndLaneOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and stop sign.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and parking space overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate overlap success, else return false.
 */
bool CalculateStopSignAndLaneOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and area.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and parking space overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate overlap success, else return false.
 */
bool CalculateAreaAndLaneOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between lane and barrier gate
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and barrier gate overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 */
bool CalculateBarrierGateOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids);

/**
 * @brief Calculate overlaps between undriveale areas and other areas.
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and parking space overlap information.
 *
 * @return Return true if calculate overlap success, else return false.
 */
bool CalculateAreaOverlaps(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *hdmap);

/**
 * @brief Calculate overlaps between junction and signal .
 *
 * @param editor_map The input editor map.
 * @param hdmap The output hdmap with lane and parking space overlap information.
 * @param lane_relation_parser The lane parser.
 * @param lane_overlap_ids The output lane overlap ids.
 *
 * @return Return true if calculate overlap success, else return false.
 */
bool CalculateJunctionAndSignalOverlaps(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *hdmap);

/**
 * @brief Determine whether a polygon overlaps a lane.
 *
 * @param lane The input lane.
 * @param polygon The input polygon.
 *
 * @return Returns true when the lane overlaps the polygon, else return false.
 */
bool IfOverlap(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        const apollo::common::math::Polygon2d &polygon);

/**
 * @brief Calculate some overlaps with special rules.
 *
 * @param editor_map The input editor map.
 * @param lane The hdmap lane.
 * @param polygon The input polygon.
 * @param type Special map element types.
 */
bool IfOverlap(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        const apollo::common::math::Polygon2d &polygon,
        std::string type);

/**
 * @brief Determine whether a segment overlaps a lane.
 *
 * @param lane The input lane.
 * @param segment The input segment.
 *
 * @return Returns true when the lane overlaps the segment, else return false.
 */
bool IfOverlap(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        const apollo::common::math::LineSegment2d &segment);

/**
 * @brief Determine whether a segment overlaps a polygon.
 *
 * @param polygon The input polygon.
 * @param segment The input segment.
 *
 * @return Returns true when the polygon overlaps the segment, else return false.
 */
bool IfOverlap(const apollo::common::math::Polygon2d &polygon, const apollo::common::math::LineSegment2d &segment);

bool IfOverlap(const apollo::common::math::Polygon2d &polygon_1, const apollo::common::math::Polygon2d &polygon_2);

/**
 * @brief Get lane overlap info when overlap is a polygon.
 *
 * @param overlap_polygon The input overlap polygon.
 * @param central_points The input central points of lane.
 * @param if_need_region If true, the lane overlap info should be include region overlap info of lane.
 * @param region_overlap_id The region overlap id if need region overlap.
 * @param lane_overlap_info The output lane overlap info.
 */
void GetOverlapInfo(
        const apollo::common::math::Polygon2d &overlap_polygon,
        const std::vector<apollo::common::math::Vec2d> &central_points,
        bool if_need_region,
        const apollo::hdmap::Id region_overlap_id,
        apollo::hdmap::LaneOverlapInfo *lane_overlap_info);

/**
 * @brief Get lane overlap info when overlap is a segment.
 *
 * @param overlap_polygon The input overlap polygon.
 * @param central_points The input central points of lane.
 * @param lane_overlap_info The output lane overlap info.
 */
void GetOverlapInfo(
        const apollo::common::math::LineSegment2d &segment,
        const std::vector<apollo::common::math::Vec2d> &central_points,
        apollo::hdmap::LaneOverlapInfo *lane_overlap_info);

/**
 * @brief Get region overlap info of lane overlap info.
 *
 * @param editor_map The input editor map.
 * @param overlap_polygon Polygons that need to be written into region overlap info.
 * @param region_overlap_id The region overlap id.
 * @param region__overlap_info The output region overlap info.
 */
void GetRegionOverlapInfo(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::common::math::Polygon2d &overlap_polygon,
        apollo::hdmap::Id &region_overlap_id,
        apollo::hdmap::RegionOverlapInfo *region_overlap_info);

/**
 * @brief Add all overlap id to lanes of hdmap.
 *
 * @param hdmap The output hdmap with all lane overlap id.
 * @param lane_overlap_ids The input overlap ids.
 */
void AddLaneOverlapId(
        apollo::hdmap::Map *hdmap,
        std::map<std::string, std::vector<apollo::hdmap::Id>> &lane_overlap_ids);

/**
 * @brief Calculate parking space start_s and end_s for reference line.
 *
 * @param central_points The central line points of the lane.
 * @param parking_space The input parking space.
 * @param start_s The output start_s
 * @param end_s The output end_s
 */
void CalculateParkingSpaceReferenceLine(
        const std::vector<apollo::common::math::Vec2d> &central_points,
        apollo::common::math::Polygon2d parking_space,
        double &start_s,
        double &end_s);

}  // namespace map_tool
}  // namespace apollo