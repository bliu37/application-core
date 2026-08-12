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

#include <math.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"

#include "cyber/common/file.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map_creator/map_tool/common/map_tool_gflags.h"
#include "modules/map_creator/map_tool/utils/util.h"

/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @struct LaneParser
 * @brief A struct to store lane information.
 */
struct LaneParser {
    // The start and end point of left lane, pair.first is start point.
    std::pair<std::string, std::string> left_start_end_;
    // The start and end point of right lane, pair.first is start point.
    std::pair<std::string, std::string> right_start_end_;
    // The ids of predecessor lane.
    std::vector<std::string> predecessor_id_;
    // The ids of successor lane.
    std::vector<std::string> successor_id_;
    // The ids of left forward neighbor lane.
    std::vector<std::string> left_forward_neighbor_id_;
    // The ids of right forward neighbor lane.
    std::vector<std::string> right_forward_neighbor_id_;
    // The ids of left reverse neighbor lane.
    std::vector<std::string> left_reverse_neighbor_id_;
    // The ids of right reverse neighbor lane
    std::vector<std::string> right_reverse_neighbor_id_;
    // The central points of lane.
    std::vector<apollo::common::math::Vec2d> central_points_;
    // Association width between central points and left lane.
    std::vector<double> left_cp_width_;
    // Association width between central points and right lane.
    std::vector<double> right_cp_width_;
    // Length between current central points and first central point.
    std::vector<double> cp_s_;
    explicit LaneParser(
            const std::pair<const std::string, const std::string> &left_start_end,
            const std::pair<std::string, std::string> &right_start_end) :
            left_start_end_(left_start_end), right_start_end_(right_start_end) {}
};

/**
 * @brief Get the boundary type by lane id.
 *
 * @param editor_map The editor map.
 * @param lane_id Includes the lane id of the query boundary.
 * @param lane_boudary_type The output boundary type.
 * @param if_right If the boundary is right or left. Turns true if right.
 *
 * @return True if success.
 */
bool GetBoudaryTypeByLaneId(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &lane_id,
        apollo::hdmap::LaneBoundaryType::Type &lane_boudary_type,
        bool if_right);

/**
 * @brief Get the boundary length by boundary points.
 *
 * @param boundary_points The Vec2d points of boundary.
 *
 * @return The length of boundary.
 */
double GetBoundaryLength(const std::vector<apollo::common::math::Vec2d> &boundary_points);

/**
 * @brief Get all point ids by lane id.
 *
 * @param editor_map The editor map.
 * @param lane_id The query lane id.
 * @param point_ids The output point ids.
 * @param if_right If the lane is right or left. Turns true if right.
 */
void GetBoundaryPointIdByLaneId(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &lane_id,
        std::vector<std::string> &point_ids,
        bool &if_right);

/**
 * @brief Get the boundary id by lane id.
 *
 * @param editor_map The editor map.
 * @param lane_id The query lane id.
 * @param if_right If the lane is right or left. Turns true if right.
 *
 * @return The boundary id.
 */
std::string GetBoundaryIdByLaneId(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &lane_id,
        bool if_right);

/**
 * @brief Get the lane length by lane id.
 *
 * @param lane_relation_parser The lane relation parser. A map that makes it
 * easier to check lane informations.
 * @param lane_id The query lane id.
 *
 * @return The length of lane.
 */
double GetLaneLength(std::map<std::string, LaneParser *> &lane_relation_parser, const std::string &lane_id);

/**
 * @brief Get the lane hdmap boundary information by lane id.
 *
 * @param editor_map The editor map.
 * @param lane_relation_parser The lane relation parser.
 * @param lane_id The query lane id.
 * @param lane The output lane with hdmap left and right boundary information.
 */
bool GetLaneBoundary(
        const apollo::map_tool::EditorMap &editor_map,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        const std::string &lane_id,
        const apollo::map_tool::Lane_Type &lane_type,
        apollo::hdmap::Lane *lane);

/**
 * @brief Get the hdmap lane boundary type by editor map lane_id.
 *
 * @param editor_map The editor map.
 * @param lane_id The query lane id.
 * @param boundary_points The query lane boundary points.
 * @param boundary The output lane boundary with boundary type.
 * @param if_right If the lane is right or left. Turns true if right.
 *
 * @return ture if get boundary type success, otherwise false.
 */
bool GetLaneBoundaryType(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &lane_id,
        const std::vector<apollo::common::math::Vec2d> &boundary_points,
        apollo::hdmap::LaneBoundary *boundary,
        bool if_right);

/**
 * @brief Get the hdmap lane central curve by editor map lane id.
 *
 * @param editor_map The editor map.
 * @param lane_relation_parser The lane relation parser.
 * @param lane_id The query lane id.
 * @param lane The output lane with hdmap central curve information.
 */
void GetLaneCentralCurve(
        const apollo::map_tool::EditorMap &editor_map,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        const std::string &lane_id,
        apollo::hdmap::Lane *lane);

/**
 * @brief Get the hdmap lane turn type by editor map lane direction.
 *
 * @param direction The query lane direction.
 * @param lane_turn The output lane turn with hdmap turn type.
 *
 * @return ture if get turn type success, otherwise false.
 */
bool GetLaneTurnType(
        const apollo::map_tool::LaneAttribute::Direction &direction,
        apollo::hdmap::Lane::LaneTurn &lane_turn);

/**
 * @brief Get the hdmap lane type by editor map.
 *
 * @param type The lane type of editor map.
 * @param lane_type The hdmap lane type.
 *
 * @return ture if get the lane type success, otherwise false.
 */
bool GetLaneType(const apollo::map_tool::LaneAttribute::LaneType &type, apollo::hdmap::Lane::LaneType &lane_type);
/**
 * @brief Get start and end points of lane boundary.
 *
 * @param editor_map The editor map.
 * @param lane The query lane.
 * @param lane_relation_parser The lane relation parser.
 */
void GetLaneStartEndPoint(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        std::map<std::string, LaneParser *> &lane_relation_parser);

/**
 * @brief Get the editor map lane by editor map lane_id.
 *
 * @param editor_map The editor map.
 * @param lane_id The query lane id.
 * @param lane The output lane.
 *
 * @return ture if get lane success, otherwise false.
 */
bool GetLaneByLaneId(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &lane_id,
        apollo::map_tool::Lane *lane);

/**
 * @brief Get lane left boundary and right boundary Vec2d points by editor map
 * lane_id.
 *
 * @param editor_map The editor map.
 * @param lane_id The query lane id.
 *
 * @return the left boundary and right boundary Vec2d points.
 */
std::pair<std::vector<apollo::common::math::Vec2d>, std::vector<apollo::common::math::Vec2d>> GetLaneByLaneId(
        const apollo::map_tool::EditorMap &editor_map,
        const std::string &lane_id);

/**
 * @brief Calculate lane relation by editor map.
 *
 * @param editor_map The editor map.
 * @param lane_relation_parser The output lane relation parser include all lane
 * relation information , such as successor lanes id, et al.
 *
 * @return ture if calculate lane relation success, otherwise false.
 */
bool CalculateLaneRelations(
        const apollo::map_tool::EditorMap &editor_map,
        std::map<std::string, LaneParser *> &lane_relation_parser);

/**
 * @brief Calculate all lane connection relationships based on lane start and
 * end points.
 *
 * @param lane_relation_parser The output lane relation parser.
 */
void CalculateLanesSuccessiveRelationships(std::map<std::string, LaneParser *> &lane_relation_parser);

/**
 * @brief Calculate one lane successor relationship by lane start and end
 * points.
 *
 * @param lane_relation_parser The output lane relation parser.
 * @param lane_id The query lane id.
 *
 * @return ture if calculate lane successor relationship success, otherwise
 * false.
 */
void CalculateLaneSuccessiveRelationship(
        std::map<std::string, LaneParser *> &lane_relation_parser,
        const std::string &lane_id);

/**
 * @brief Calculate all lane neighbor relationships based on lane start and
 * end points.
 *
 * @param lane_relation_parser The output lane relation parser.
 */
void CalculateLanesNeighborRelationships(std::map<std::string, LaneParser *> &lane_relation_parser);

/**
 * @brief Calculate one lane neighbor relationship by lane start and end
 * points.
 *
 * @param lane_relation_parser The output lane relation parser.
 * @param lane_id The query lane id.
 *
 * @return ture if calculate lane neighbor relationship success, otherwise
 * false.
 */
void CalculateLaneNeighborRelationships(
        std::map<std::string, LaneParser *> &lane_relation_parser,
        const std::string &lane_id);

/**
 * @brief Calculate hdmap lane central points by lane start and end
 * points.
 *
 * @param left_start_point The input left start point.
 * @param right_start_point The input right start point.
 * @param central_point The starting point of centerline.
 * @param step_length Centerline step length.
 * @param lane The input lane boundary points.
 * @param lane_parser The output lane parser include center point.
 */
void CalculateCentralPointsOfLane(
        apollo::common::math::Vec2d &left_start_point,
        apollo::common::math::Vec2d &right_start_point,
        apollo::common::math::Vec2d &central_point,
        float step_length,
        const std::pair<std::vector<apollo::common::math::Vec2d>, std::vector<apollo::common::math::Vec2d>> &lane,
        LaneParser &lane_parser);

/**
 * @brief Calculate the center point of the curve.
 *
 * @param central_start_point The start point of central lane.
 * @param center_end_point The end point of central lane.
 * @param lane The curve info.
 * @param lane_parser The lane parser of curve central information.
 */
void CalculateCentralPointsOfCurveLane(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::common::math::Vec2d &central_start_point,
        const apollo::common::math::Vec2d &central_end_point,
        const apollo::map_tool::Lane &lane,
        LaneParser &lane_parser,
        double &step);

/**
 * @brief Calculate all hdmap lane central points information.
 *
 * @param editor_map The editor map.
 * @param lane_relation_parser The output lane relation parser.
 */
void CalculateCentralPoints(
        const apollo::map_tool::EditorMap &editor_map,
        std::map<std::string, LaneParser *> &lane_relation_parser);

/**
 * @brief Fill in the successor relationship and neighbor relationship of a hdmap lane.
 *
 * @param lane The output hdmap lane.
 * @param lane_parser The lane relation parser.
 */
void FillLaneNeighborSuccessorRelationships(apollo::hdmap::Lane *lane, apollo::map_tool::LaneParser *lane_parser);

/**
 * @brief Calculate the interpolation point of the curve using Bessel's formula. The interpolation point p(x,y) can be
 * calculated using the following formula:
 * x(t) = (1-t)^3*p0x + 3(1-t)^2*t*p1x + 3*(1-t)^2*t^2*p2x + t^3*p3x,
 * y(t) = (1-t)^3*p0y + 3(1-t)^2*t*p1y + 3*(1-t)^2*t^2*p2y + t^3*p3y, t(0,1)
 *
 * @param start The p0 point.
 * @param end The p3 point.
 * @param control_1 The p1 point.
 * @param control_2 The p2 point.
 * @param t The sampling parameters.
 *
 * @return The interpolation point.
 */
apollo::map_tool::Position CalculateCurveInterpolationPoint(
        const apollo::map_tool::Position &start,
        const apollo::map_tool::Position &end,
        const apollo::map_tool::Position &control_1,
        const apollo::map_tool::Position &control_2,
        double &t);

/**
 * @brief Get interpolation points on a curve
 *
 * @param start The start point of curve.
 * @param end The end point of curve.
 * @param control_1 The first control point.
 * @param control_2 The second control point.
 * @param points The all interpolation points of the curve.
 */
void GetInterpolationCurvePoints(
        const apollo::map_tool::Position &start,
        const apollo::map_tool::Position &end,
        const apollo::map_tool::Position &control_1,
        const apollo::map_tool::Position &control_2,
        std::vector<apollo::map_tool::Position> *points,
        double &step);

/**
 * @brief Add curve interpolation point to editor map boundary of the curve.
 *
 * @param boundary_id The boundary id.
 * @param interpolation_points All interpolation points of curve.
 * @param eidtor_map The output eiitor map with interpolation points.
 */
void AddInterpolationToCurveBoundary(
        const std::string &boundary_id,
        const std::vector<apollo::map_tool::Position> &interpolation_points,
        apollo::map_tool::EditorMap *editor_map);

}  // namespace map_tool
}  // namespace apollo
