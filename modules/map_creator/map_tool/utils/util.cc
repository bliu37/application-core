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
#include "modules/map_creator/map_tool/utils/util.h"

namespace apollo {
namespace map_tool {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using Json = nlohmann::json;
using apollo::tile_map_images_creator::GridCell;
using apollo::tile_map_images_creator::GridCellSingle;
using apollo::tile_map_images_creator::GridOption;
using google::protobuf::util::MessageToJsonString;
namespace {

using Json = nlohmann::json;
using google::protobuf::util::MessageToJsonString;

google::protobuf::util::JsonPrintOptions JsonOptions(
    std::vector<std::string>& options) {
  google::protobuf::util::JsonPrintOptions json_option;
  for (const auto& option : options) {
    if (option == "primitive_fields") {
      json_option.always_print_primitive_fields = true;
    } else if (option == "enums_as_ints") {
      json_option.always_print_enums_as_ints = true;
    } else if (option == "add_whitespace") {
      json_option.add_whitespace = true;
    } else if (option == "preserve_proto_field_names") {
      json_option.preserve_proto_field_names = true;
    }
  }
  return json_option;
}

}  // namespace

std::vector<LineSegment2d> GetLineSegment2ds(const std::vector<Vec2d>& points) {
  std::vector<LineSegment2d> line_segments;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    line_segments.emplace_back(points[i], points[i + 1]);
  }
  return line_segments;
}

std::vector<Vec2d> GetVec2dPoints(const apollo::map_tool::EditorMap& editor_map,
                                  std::vector<std::string> point_ids) {
  std::vector<Vec2d> points;
  std::unordered_map<std::string, apollo::map_tool::Point> point_map;
  for (const auto& point : editor_map.point()) {
    point_map[point.id()] = point;
  }
  for (const auto& point_id : point_ids) {
    points.push_back(Vec2d(point_map[point_id].position().x(),
                           point_map[point_id].position().y()));
  }
  return points;
}

bool GetNearestPointIndex(const std::vector<Vec2d>& points, const Vec2d& point,
                          std::size_t& point_index) {
  for (size_t i = 0; i < points.size() - 1; ++i) {
    LineSegment2d line_segment(points[i], points[i + 1]);
    if (line_segment.IsPointIn(point)) {
      point_index = i;
      return true;
    }
  }
  return false;
}

Vec2d GetNextPointFixedDistance(const std::vector<Vec2d>& points,
                                const Vec2d& start_point,
                                const double& distance) {
  // Find the starting point of the line segment where start_point is located.
  // The initial value of the remain distance is distance.
  // If the distance between the line segments is less than the remaining
  // distance, enter the multi-segment loop calculation.
  std::size_t start_index = 0;
  double remain_distance = distance;
  if (!GetNearestPointIndex(points, start_point, start_index)) {
    AERROR << "Get nearest point index failed!";
  }
  double segment_distance = points[start_index + 1].DistanceTo(start_point);
  if (segment_distance >= remain_distance) {
    double fraction = distance / segment_distance;
    Vec2d next_point =
        start_point + fraction * (points[start_index + 1] - start_point);
    return next_point;
  } else if ((start_index + 1) == (points.size() - 1)) {
    return points.back();
  }
  remain_distance -= segment_distance;
  for (size_t i = start_index + 1; i < points.size() - 1; ++i) {
    const Vec2d& p1 = points[i];
    const Vec2d& p2 = points[i + 1];
    double segment_distance = p2.DistanceTo(p1);
    if (segment_distance >= remain_distance) {
      double fraction = remain_distance / segment_distance;
      Vec2d next_point = p1 + fraction * (p2 - p1);
      return next_point;
    } else {
      remain_distance -= segment_distance;
    }
  }
  return points.back();
}

Vec2d GetNextPointFixedDistance(const Vec2d& point, LineSegment2d& line_segment,
                                const double& distance) {
  Vec2d next_point = point + distance * line_segment.unit_direction();
  return next_point;
}

Vec2d GetPointDistanceToLane(const std::vector<Vec2d>& lane, const Vec2d& point,
                             double& distance) {
  LineSegment2d line_min(lane[0], lane[1]);
  std::size_t index = 0;
  double distance_min = line_min.DistanceTo(point);
  for (std::size_t i = 1; i < lane.size() - 1; ++i) {
    LineSegment2d line_temp(lane[i], lane[i + 1]);
    double distance_tmp = line_temp.DistanceTo(point);
    if (distance_tmp < distance_min) {
      distance_min = distance_tmp;
      index = i;
    }
  }
  LineSegment2d line_segment(lane[index], lane[index + 1]);
  Vec2d foot_point;
  distance = line_segment.GetPerpendicularFoot(point, &foot_point);
  if (!line_segment.IsPointIn(foot_point)) {
    double distance_to_first_point = point.DistanceTo(lane[index]);
    double distance_to_second_point = point.DistanceTo(lane[index + 1]);
    if (distance_to_first_point >= distance_to_second_point) {
      return lane[index + 1];
    } else {
      return lane[index];
    }
  }
  return foot_point;
}

bool GetProtoFromJson(const nlohmann::json& json,
                      google::protobuf::Message* message) {
  using google::protobuf::util::JsonParseOptions;
  using google::protobuf::util::JsonStringToMessage;
  JsonParseOptions options;
  options.ignore_unknown_fields = true;
  return (JsonStringToMessage(json.dump(), message, options).ok());
}

Json ProtoToJson(const google::protobuf::Message& proto,
                 std::vector<std::string>& options) {
  static const auto kJsonOption = JsonOptions(options);
  std::string json_string;
  const auto status = MessageToJsonString(proto, &json_string, kJsonOption);
  Json json_obj = Json::parse(json_string);
  return json_obj;
}

apollo::hdmap::Polygon GetPolygonByBoundaryId(
    const apollo::map_tool::EditorMap& editor_map,
    const std::string& boundary_id) {
  apollo::hdmap::Polygon polygon;
  std::vector<std::string> point_ids;
  double map_offset_x = editor_map.basemap_center().x();
  double map_offset_y = editor_map.basemap_center().y();
  GetPointIdByBoundaryId(editor_map, boundary_id, point_ids);
  std::map<std::string, apollo::map_tool::Point> point_map;
  for (const auto& point : editor_map.point()) {
    point_map[point.id()] = point;
  }
  for (const auto& point_id : point_ids) {
    apollo::common::PointENU* point = polygon.add_point();
    point->set_x(point_map[point_id].position().x() + map_offset_x);
    point->set_y(point_map[point_id].position().y() + map_offset_y);
    point->set_z(point_map[point_id].position().z());
  }
  return polygon;
}

apollo::hdmap::Polygon GetPolygonByPoints(
    const apollo::map_tool::EditorMap& editor_map,
    const std::vector<Vec2d>& points) {
  apollo::hdmap::Polygon polygon;
  double map_offset_x = editor_map.basemap_center().x();
  double map_offset_y = editor_map.basemap_center().y();
  for (const auto& point : points) {
    apollo::common::PointENU* point_pb = polygon.add_point();
    point_pb->set_x(point.x() + map_offset_x);
    point_pb->set_y(point.y() + map_offset_y);
    point_pb->set_z(0);
  }
  return polygon;
}

void GetBoundaryCurve(const std::vector<Vec2d>& boundary_points,
                      apollo::hdmap::Curve* curve, Vec2d& map_offset) {
  double length = 0;
  apollo::hdmap::CurveSegment* segment = curve->add_segment();
  apollo::hdmap::LineSegment* line_segment = segment->mutable_line_segment();
  apollo::common::PointENU* pt = line_segment->add_point();
  pt->set_x(boundary_points[0].x() + map_offset.x());
  pt->set_y(boundary_points[0].y() + map_offset.y());
  for (size_t i = 1; i < boundary_points.size(); ++i) {
    apollo::common::PointENU* pt = line_segment->add_point();
    pt->set_x(boundary_points[i].x() + map_offset.x());
    pt->set_y(boundary_points[i].y() + map_offset.y());
    length += boundary_points[i].DistanceTo(boundary_points[i - 1]);
  }
  LineSegment2d first_line(boundary_points[0], boundary_points[1]);
  apollo::common::PointENU* start_position = segment->mutable_start_position();
  start_position->set_x(boundary_points[0].x() + map_offset.x());
  start_position->set_y(boundary_points[0].y() + map_offset.y());
  segment->set_heading(first_line.heading());
  segment->set_s(0);
  segment->set_length(length);
}

void GetStopLineCurve(const apollo::map_tool::EditorMap& editor_map,
                      const apollo::map_tool::StopLine& stop_line,
                      apollo::hdmap::Curve* curve) {
  std::vector<std::string> point_ids;
  std::vector<Vec2d> points;
  GetPointIdByBoundaryId(editor_map, stop_line.boundary_id(), point_ids);
  std::unordered_map<std::string, apollo::map_tool::Point> point_map;
  for (const auto& point : editor_map.point()) {
    point_map[point.id()] = point;
  }
  for (const auto& point_id : point_ids) {
    points.push_back(Vec2d(point_map[point_id].position().x(),
                           point_map[point_id].position().y()));
  }
  Vec2d map_offset(editor_map.basemap_center().x(),
                   editor_map.basemap_center().y());
  GetBoundaryCurve(points, curve, map_offset);
}

bool GetStopLineById(const apollo::map_tool::EditorMap& editor_map,
                     const std::string& id,
                     apollo::map_tool::StopLine& stop_line) {
  auto iter =
      std::find_if(editor_map.stop_line().begin(), editor_map.stop_line().end(),
                   [&](const auto& iter) { return iter.id() == id; });
  if (iter == editor_map.stop_line().end()) {
    AERROR << "Can't find stop line of id: " << id;
    return false;
  }
  stop_line = apollo::map_tool::StopLine(*iter);
  return true;
}

void GetPointIdByBoundaryId(const apollo::map_tool::EditorMap& editor_map,
                            const std::string& boundary_id,
                            std::vector<std::string>& point_ids) {
  apollo::map_tool::Boundary boundary;
  GetBoundaryByBoundaryId(editor_map, boundary_id, boundary);
  for (auto& point_id : boundary.point_id()) {
    point_ids.push_back(point_id);
  }
}

bool GetBoundaryByBoundaryId(const apollo::map_tool::EditorMap& editor_map,
                             const std::string& boundary_id,
                             apollo::map_tool::Boundary& boundary) {
  auto iter =
      std::find_if(editor_map.boundary().begin(), editor_map.boundary().end(),
                   [&](const auto& iter) { return iter.id() == boundary_id; });

  if (iter != editor_map.boundary().end()) {
    boundary = apollo::map_tool::Boundary(*iter);
    return true;
  }
  return false;
}

bool GetPointByPointId(const apollo::map_tool::EditorMap& editor_map,
                       const std::string& point_id,
                       apollo::map_tool::Point& point) {
  auto iter =
      std::find_if(editor_map.point().begin(), editor_map.point().end(),
                   [&](const auto& iter) { return iter.id() == point_id; });

  if (iter != editor_map.point().end()) {
    point = apollo::map_tool::Point(*iter);
    return true;
  }
  return false;
}

bool GetIntersect(const LineSegment2d& line,
                  const std::vector<LineSegment2d>& line_segments,
                  Vec2d* const point) {
  for (const auto& line_segment : line_segments) {
    if (line_segment.GetIntersect(line, point)) {
      return true;
    }
  }
  return false;
}

bool GetIntersect(const Polygon2d& polygon,
                  const std::vector<LineSegment2d>& line_segments,
                  std::vector<Vec2d>* intersects) {
  for (const auto& line_segment : line_segments) {
    for (const auto& polygon_line : polygon.line_segments()) {
      if (polygon_line.HasIntersect(line_segment)) {
        Vec2d intersect;
        polygon_line.GetIntersect(line_segment, &intersect);
        intersects->push_back(intersect);
      }
      if (intersects->size() >= 2) break;
    }
  }
  if (intersects->size() < 2) {
    AERROR << "Can't find intersect between input polygon and line segments.";
    return false;
  }
  return true;
}

double GetDistanceTo(const std::vector<LineSegment2d>& line_segments,
                     const Vec2d& point) {
  double length = 0;
  for (const auto& line_segment : line_segments) {
    if (line_segment.IsPointIn(point)) {
      length += line_segment.start().DistanceTo(point);
      break;
    }
    length += line_segment.length();
  }
  return length;
}

double GetGroundHeight(const double& x, const double& y, const double& radis) {
  if (!cyber::common::PathExists(FLAGS_curr_base_map_dir + "/map_bin/")) {
    return 0.0;
  }
  double height = std::numeric_limits<double>::max();
  // Data comes from image_creator_conf.pb.txt, indicates that a grid is 0.03125
  // meters.
  apollo::tile_map_images_creator::ImagesCreatorConf conf;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_default_images_creator_config_path, &conf)) {
    return 0.0;
  }
  // TODO(fanyueqiao): matrix_size is also read from the configuration file
  double resolution = conf.matrix_generator_conf().matrix_resolution();
  unsigned int matrix_size = 1024;
  std::shared_ptr<GridOption> opt_ptr = std::make_shared<GridOption>();
  opt_ptr->set_matrix_size(matrix_size);
  opt_ptr->set_resolution_id(4);
  opt_ptr->set_resolution(resolution);
  opt_ptr->set_zone_id(0);
  opt_ptr->set_min_x(0.0);
  opt_ptr->set_min_y(0.0);
  opt_ptr->set_min_z(-10000.0);
  apollo::tile_map_images_creator::GridMatrix grid_matrix;
  grid_matrix.Init(opt_ptr);
  // 32 is matrix size multiplied by resultion.
  apollo::tile_map_images_creator::MatrixIndex matrix_index(x / 32, y / 32);
  grid_matrix.set_index(matrix_index);
  grid_matrix.LoadBinaryMatrixFile(FLAGS_curr_base_map_dir + "/map_bin/");
  unsigned int rows = grid_matrix.get_rows();
  unsigned int cols = grid_matrix.get_cols();
  int row = std::fmod(x / resolution, 1024);
  int col = std::fmod(y / resolution, 1024);
  // Find the minimum z value in the range 0.3m by 0.3m.
  unsigned int row_start = row - radis;
  unsigned int row_end = row + radis;
  unsigned int col_start = col - radis;
  unsigned int col_end = col + radis;
  row_start = row_start > 0 ? row_start : 0;
  row_end = row_end < matrix_size ? row_end : matrix_size;
  col_start = col_start > 0 ? col_start : 0;
  col_end = col_end < matrix_size ? col_end : matrix_size;
  for (unsigned int i = row_start; i <= row_end; i++) {
    for (unsigned int j = col_start; j <= col_end; j++) {
      GridCell grid_cell = grid_matrix.get_const_map_cell(i, j);
      std::vector<GridCellSingle> single_cells;
      grid_cell.GetSingleCells(single_cells);
      if (!single_cells.empty()) {
        height = single_cells[0].altitude_ < height ? single_cells[0].altitude_
                                                    : height;
      }
    }
  }
  if (std::numeric_limits<double>::max() == height) {
    height = 0;
  }
  return height;
}

}  // namespace map_tool
}  // namespace apollo
