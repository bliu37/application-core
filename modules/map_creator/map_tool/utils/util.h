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
#include <cxxabi.h>

#include <vector>

#include "google/protobuf/util/json_util.h"
#include "nlohmann/json.hpp"

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/common_msgs/map_msgs/map_overlap.pb.h"
#include "modules/map_creator/map_tool/proto/editor_map.pb.h"
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map_creator/map_tool/common/map_tool_gflags.h"
#include "modules/map_creator/map_tool/utils/util.h"
#include "modules/map_creator/tile_map_images_creator/common/image_creator_flags.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_cell.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_matrix.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_option.h"
/**
 * @namespace apollo::map_tool
 * @brief apollo::map_tool
 */
namespace apollo {
namespace map_tool {

/**
 * @brief Calculate the point on a boundary line at a fixed distance from the
 * starting point.
 *
 * @param points The points of a boundary line.
 * @param start_point The point to calculate.
 * @param distance The fixed distance.
 *
 * @return The next point.
 */
apollo::common::math::Vec2d GetNextPointFixedDistance(
        const std::vector<apollo::common::math::Vec2d>& points,
        const apollo::common::math::Vec2d& start_point,
        const double& distance);

/**
 * @brief Calculate the point on the ray where the line segment lies at a fixed
 * distance from the starting point.
 *
 * @param line_segment The line segment.
 * @param point The starting point for calculation.
 * @param distance The fixed distance.
 *
 * @return The next point.
 */
apollo::common::math::Vec2d GetNextPointFixedDistance(
        const apollo::common::math::Vec2d& point,
        apollo::common::math::LineSegment2d& line_segment,
        const double& distance);

/**
 * @brief Concatenate discrete points into line segments.
 *
 * @param points The discrete points.
 *
 * @return The line segments.
 */
std::vector<apollo::common::math::LineSegment2d> GetLineSegment2ds(
        const std::vector<apollo::common::math::Vec2d>& points);

/**
 * @brief Find the starting point of the line segment where a point lies
 *
 * @param points The discrete points of a boundary line.
 * @param point The point to find.
 * @param point_index The index of the starting point.
 *
 * @return True if the point is found, otherwise false.
 */
bool GetNearestPointIndex(
        const std::vector<apollo::common::math::Vec2d>& points,
        const apollo::common::math::Vec2d& point,
        std::size_t& point_index);

/**
 * @brief Calculate the distance from a point to a line segment and obtain its
 * vertical foot. If the vertical foot from the point to the line segment
 * exceeds the line segment, take the end point of the line segment.
 *
 * @param line_segment The query line segment.
 * @param point The query point.
 * @param distance The output distance.
 *
 * @return The vertical foot.
 */
apollo::common::math::Vec2d GetPointDistanceToLane(
        const std::vector<apollo::common::math::Vec2d>& lane,
        const apollo::common::math::Vec2d& point,
        double& distance);

/**
 * @brief Get the hdmap polygon of the editor map element by boundary id.
 *
 * @param editor_map  The editor map.
 * @param boundary_id The boundary id of the editor map element.
 *
 * @return The polygon of map element.
 */
apollo::hdmap::Polygon GetPolygonByBoundaryId(
        const apollo::map_tool::EditorMap& editor_map,
        const std::string& boundary_id);

/**
 * @brief Get the hdmap polygon from points
 *
 * @param editor_map The editor_map
 * @param points The Vec2d points of polygon
 *
 * @return The polygon of input points.
 */
apollo::hdmap::Polygon GetPolygonByPoints(
        const apollo::map_tool::EditorMap& editor_map,
        const std::vector<apollo::common::math::Vec2d>& points);

/**
 * @brief Get all point ids by boundary id.
 *
 * @param editor_map The editor map.
 * @param boundary_id The query boundary id.
 * @param point_ids The output point ids.
 */
void GetPointIdByBoundaryId(
        const apollo::map_tool::EditorMap& editor_map,
        const std::string& boundary_id,
        std::vector<std::string>& point_ids);

/**
 * @brief Get the boundary hdmap curve id by boundary point.
 *
 * @param boundary_points The Vec2d points of boundary.
 * @param curve The output hdmap curvet.
 */
void GetBoundaryCurve(
        const std::vector<apollo::common::math::Vec2d>& boundary_points,
        apollo::hdmap::Curve* curve,
        apollo::common::math::Vec2d& map_offset);

/**
 * @brief Get stop line by id.
 *
 * @param id The input stop line id.
 * @param stop_line The input stop line.
 *
 * @return Return true id get stop line success.
 */
bool GetStopLineById(
        const apollo::map_tool::EditorMap& editor_map,
        const std::string& id,
        apollo::map_tool::StopLine& stop_line);
/**
 * @brief Get stopline curve.
 *
 * @param editor_map The input editor map.
 * @param stop_line The input stop line.
 * @param curve The output curve calculated from stop line.
 */
void GetStopLineCurve(
        const apollo::map_tool::EditorMap& editor_map,
        const apollo::map_tool::StopLine& stop_line,
        apollo::hdmap::Curve* curve);
/**
 * @brief Get the editor map boundary by editor map lane_id.
 *
 * @param editor_map The editor map.
 * @param boundary_id The query boundary id.
 * @param boundary The output boundary.
 *
 * @return ture if get boundary success, otherwise false.
 */
bool GetBoundaryByBoundaryId(
        const apollo::map_tool::EditorMap& editor_map,
        const std::string& boundary_id,
        apollo::map_tool::Boundary& boundary);

/**
 * @brief Get point by point id.
 *
 * @param editor_map The editor map.
 * @param point_id The input point id.
 * @param point The output editor map point.
 */
bool GetPointByPointId(
        const apollo::map_tool::EditorMap& editor_map,
        const std::string& point_id,
        apollo::map_tool::Point& point);

/**
 * @brief Parses the content of the json to the proto.
 * @param json The json to parse to protobuf.
 * @param message The proto to carry the parsed content.
 * @return If the action is successful.
 */
bool GetProtoFromJson(const nlohmann::json& json, google::protobuf::Message* message);

/**
 * @brief  Convert proto to a json string with options.
 * @return A json object from a proto.
 */
nlohmann::json ProtoToJson(const google::protobuf::Message& proto, std::vector<std::string>& options);

/**
 * @brief Get Vec2d points from point ids
 *
 * @param editor_map The input editor map.
 * @param points The input point ids
 *
 * @return The vector of all Vec2d points from point ids.
 */
std::vector<apollo::common::math::Vec2d> GetVec2dPoints(
        const apollo::map_tool::EditorMap& editor_map,
        std::vector<std::string> point_ids);

/**
 * @brief Get intersect point between a segment and a boundary segments.
 *
 * @param line The input segment.
 * @param line_segments The segments of a boundary.
 * @param point The output intersect point.
 *
 * @return Returns true if the input segment has intersect point with the boundary.
 */
bool GetIntersect(
        const apollo::common::math::LineSegment2d& line,
        const std::vector<apollo::common::math::LineSegment2d>& line_segments,
        apollo::common::math::Vec2d* const point);

/**
 * @brief Get intersect points between a boundary and a polygon.
 *
 * @param polygon The input polygon.
 * @param line_segments The segments of a boundary.
 * @param intersects The output intersect points.
 *
 * @return Return true if the input segments has intersect points with the polygon.
 */
bool GetIntersect(
        const apollo::common::math::Polygon2d& polygon,
        const std::vector<apollo::common::math::LineSegment2d>& line_segments,
        std::vector<apollo::common::math::Vec2d>* intersects);

/**
 * @brief Get the distance between a point on a boundary.
 *
 * @param line_segments The segments of a boundary.
 * @param point The input point.
 *
 * @return Return the distance.
 */
double GetDistanceTo(
        const std::vector<apollo::common::math::LineSegment2d>& line_segments,
        const apollo::common::math::Vec2d& point);

/**
 * @brief Get ground height.
 *
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @param radis The search range of input (x,y) coordinate.
 *
 * @return The height of the ground where the signal light is located.
 */
double GetGroundHeight(const double& x, const double& y, const double& radis);

/**
 * @brief Get map element unique id.
 *
 * @param id the input editor map id.
 *
 * @return The HD map unique map element id.
 */
template <typename Base>
apollo::hdmap::Id GetMapElementId(const std::string& id) {
    int status = 0;
    std::string base_class_name = abi::__cxa_demangle(typeid(Base).name(), 0, 0, &status);
    apollo::hdmap::Id id_pb;
    size_t last_colon_pos = base_class_name.rfind("::");
    if (last_colon_pos != std::string::npos) {
        std::string class_name = base_class_name.substr(last_colon_pos + 2);
        id_pb.set_id(class_name + "_" + id);
    } else {
        id_pb.set_id(id);
    }
    return id_pb;
}

/**
 * @brief Get polygon2d from input editor map elements (except lane).
 *
 * @param editor_map The input editor map.
 * @param element The input map element.
 *
 * @return The output polygon of element.
 */
template <typename T>
apollo::common::math::Polygon2d GetPolygon2d(const apollo::map_tool::EditorMap& editor_map, const T& element) {
    std::vector<std::string> elements_point_ids;
    std::vector<apollo::common::math::Vec2d> elements_points;
    apollo::common::math::Polygon2d element_polygon;
    GetPointIdByBoundaryId(editor_map, element.boundary_id(), elements_point_ids);
    elements_points = GetVec2dPoints(editor_map, elements_point_ids);
    apollo::common::math::Polygon2d::ComputeConvexHull(elements_points, &element_polygon);
    return element_polygon;
}

/**
 * @brief Get polygon2d from input editor map lane.
 *
 * @param editor_map The input editor map.
 * @param element The input lane element.
 *
 * @return The output polygon of lane polygon.
 */
template <>
inline apollo::common::math::Polygon2d GetPolygon2d<apollo::map_tool::Lane>(
        const apollo::map_tool::EditorMap& editor_map,
        const apollo::map_tool::Lane& lane) {
    std::vector<std::string> left_boundary_point_ids;
    std::vector<std::string> right_boundary_point_ids;
    std::vector<apollo::common::math::Vec2d> left_points;
    std::vector<apollo::common::math::Vec2d> right_points;
    apollo::common::math::Polygon2d lane_polygon;
    GetPointIdByBoundaryId(editor_map, lane.left_boundary_id(), left_boundary_point_ids);
    GetPointIdByBoundaryId(editor_map, lane.right_boundary_id(), right_boundary_point_ids);
    left_points = GetVec2dPoints(editor_map, left_boundary_point_ids);
    right_points = GetVec2dPoints(editor_map, right_boundary_point_ids);
    // Process the vertices of the road so that they form a polygon.
    std::vector<apollo::common::math::Vec2d> lane_polygon_points(left_points);
    if (lane.left_boundary_reverse() == lane.right_boundary_reverse()) {
        lane_polygon_points.insert(lane_polygon_points.end(), right_points.rbegin(), right_points.rend());
    }
    apollo::common::math::Polygon2d::ComputeConvexHull(lane_polygon_points, &lane_polygon);
    return lane_polygon;
}

}  // namespace map_tool
}  // namespace apollo
