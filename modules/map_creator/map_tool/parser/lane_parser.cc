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
#include "modules/map_creator/map_tool/parser/lane_parser.h"

#include <algorithm>
#include <unordered_map>
namespace apollo {
namespace map_tool {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;
using apollo::map_tool::LaneAttribute;

bool GetLaneBoundary(const apollo::map_tool::EditorMap &editor_map,
                     std::map<std::string, LaneParser *> &lane_relation_parser,
                     const std::string &lane_id,
                     const apollo::map_tool::Lane_Type &lane_type,
                     apollo::hdmap::Lane *lane) {
  apollo::hdmap::LaneBoundary *left_boundary = lane->mutable_left_boundary();
  apollo::hdmap::LaneBoundary *right_boundary = lane->mutable_right_boundary();
  apollo::hdmap::Curve *left_curve = left_boundary->mutable_curve();
  apollo::hdmap::Curve *right_curve = right_boundary->mutable_curve();
  std::pair<std::vector<Vec2d>, std::vector<Vec2d>> lane_boundary =
      GetLaneByLaneId(editor_map, lane_id);
  Vec2d map_offset(editor_map.basemap_center().x(),
                   editor_map.basemap_center().y());

  GetBoundaryCurve(lane_boundary.first, left_curve, map_offset);
  GetBoundaryCurve(lane_boundary.second, right_curve, map_offset);
  right_boundary->set_virtual_(true);
  right_boundary->set_length(GetBoundaryLength(lane_boundary.second));
  left_boundary->set_virtual_(true);
  left_boundary->set_length(GetBoundaryLength(lane_boundary.first));
  if (!GetLaneBoundaryType(editor_map, lane_id, lane_boundary.first,
                           left_boundary, false)) {
    return false;
  }
  if (!GetLaneBoundaryType(editor_map, lane_id, lane_boundary.second,
                           right_boundary, true)) {
    return false;
  }
  return true;
}

void GetLaneCentralCurve(
    const apollo::map_tool::EditorMap &editor_map,
    std::map<std::string, LaneParser *> &lane_relation_parser,
    const std::string &lane_id, apollo::hdmap::Lane *lane) {
  apollo::hdmap::Curve *lane_curve = lane->mutable_central_curve();
  std::vector<Vec2d> center_points =
      lane_relation_parser[lane_id]->central_points_;
  Vec2d map_offset(editor_map.basemap_center().x(),
                   editor_map.basemap_center().y());
  GetBoundaryCurve(center_points, lane_curve, map_offset);
}

bool GetLaneBoundaryType(const apollo::map_tool::EditorMap &editor_map,
                         const std::string &lane_id,
                         const std::vector<Vec2d> &boundary_points,
                         apollo::hdmap::LaneBoundary *boundary, bool if_right) {
  // TODO(all): A boundary in a high-precision map has multiple boundary types.
  // Here, a boundary in the edited map has only one attribute, and each line
  // segment is not judged for the time being.
  apollo::hdmap::LaneBoundaryType::Type boundary_type;
  if (!GetBoudaryTypeByLaneId(editor_map, lane_id, boundary_type, if_right)) {
    return false;
  }
  apollo::hdmap::LaneBoundaryType *lane_boundary_type =
      boundary->add_boundary_type();
  lane_boundary_type->set_s(0);
  lane_boundary_type->add_types(boundary_type);
  return true;
}

double GetLaneLength(std::map<std::string, LaneParser *> &lane_relation_parser,
                     const std::string &lane_id) {
  double lane_length = lane_relation_parser[lane_id]->cp_s_.back();
  return lane_length;
}

bool GetBoudaryTypeByLaneId(
    const apollo::map_tool::EditorMap &editor_map, const std::string &lane_id,
    apollo::hdmap::LaneBoundaryType::Type &lane_boudary_type, bool if_right) {
  std::string boundary_id =
      GetBoundaryIdByLaneId(editor_map, lane_id, if_right);
  apollo::map_tool::Boundary boundary;
  GetBoundaryByBoundaryId(editor_map, boundary_id, boundary);
  if (boundary.attr().type() ==
      apollo::map_tool::BoundaryAttribute::DOTTED_YELLOW) {
    lane_boudary_type = apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW;
  } else if (boundary.attr().type() ==
             apollo::map_tool::BoundaryAttribute::DOTTED_WHITE) {
    lane_boudary_type = apollo::hdmap::LaneBoundaryType::DOTTED_WHITE;
  } else if (boundary.attr().type() ==
             apollo::map_tool::BoundaryAttribute::SOLID_WHITE) {
    lane_boudary_type = apollo::hdmap::LaneBoundaryType::SOLID_WHITE;
  } else if (boundary.attr().type() ==
             apollo::map_tool::BoundaryAttribute::SOLID_YELLOW) {
    lane_boudary_type = apollo::hdmap::LaneBoundaryType::SOLID_YELLOW;
  } else if (boundary.attr().type() ==
             apollo::map_tool::BoundaryAttribute::UNKNOWN) {
    lane_boudary_type = apollo::hdmap::LaneBoundaryType::UNKNOWN;
  } else {
    AERROR << "Unknown boundary type: " << boundary.attr().type();
    return false;
  }
  return true;
}

bool GetLaneType(const LaneAttribute::LaneType &type,
                 apollo::hdmap::Lane::LaneType &lane_type) {
  if (type == LaneAttribute::BIKING) {
    lane_type = apollo::hdmap::Lane::BIKING;
    return true;
  } else if (type == LaneAttribute::SHARED) {
    lane_type = apollo::hdmap::Lane::CITY_DRIVING;
    return true;
  } else if (type == LaneAttribute::CITY_DRIVING) {
    lane_type = apollo::hdmap::Lane::CITY_DRIVING;
    return true;
  } else {
    AERROR << "Unknown lane type: " << type;
    return false;
  }
  return true;
}

double GetBoundaryLength(
    const std::vector<apollo::common::math::Vec2d> &boundary_points) {
  int length = 0;
  for (size_t i = 0; i < boundary_points.size() - 1; ++i) {
    double distance = boundary_points[i].DistanceTo(boundary_points[i + 1]);
    length += distance;
  }
  return length;
}

bool GetLaneTurnType(
    const apollo::map_tool::LaneAttribute::Direction &direction,
    apollo::hdmap::Lane::LaneTurn &lane_turn) {
  if (direction == apollo::map_tool::LaneAttribute::NO_TURN) {
    lane_turn = apollo::hdmap::Lane::NO_TURN;
  } else if (direction == apollo::map_tool::LaneAttribute::LEFT_TURN) {
    lane_turn = apollo::hdmap::Lane::LEFT_TURN;
  } else if (direction == apollo::map_tool::LaneAttribute::RIGHT_TURN) {
    lane_turn = apollo::hdmap::Lane::RIGHT_TURN;
  } else if (direction == apollo::map_tool::LaneAttribute::U_TURN) {
    lane_turn = apollo::hdmap::Lane::U_TURN;
  } else {
    AERROR << "Unknown lane turn type: " << direction;
    return false;
  }
  return true;
}

bool CalculateLaneRelations(
    const apollo::map_tool::EditorMap &editor_map,
    std::map<std::string, LaneParser *> &lane_relation_parser) {
  lane_relation_parser.clear();
  for (const auto &lane : editor_map.lane()) {
    GetLaneStartEndPoint(editor_map, lane, lane_relation_parser);
  }
  CalculateLanesSuccessiveRelationships(lane_relation_parser);
  CalculateLanesNeighborRelationships(lane_relation_parser);
  return true;
}

void GetLaneStartEndPoint(
    const apollo::map_tool::EditorMap &editor_map,
    const apollo::map_tool::Lane &lane,
    std::map<std::string, LaneParser *> &lane_relation_parser) {
  std::vector<std::string> right_point_ids;
  std::vector<std::string> left_point_ids;
  // right boundary
  GetPointIdByBoundaryId(editor_map, lane.right_boundary_id(), right_point_ids);
  // left boundary
  GetPointIdByBoundaryId(editor_map, lane.left_boundary_id(), left_point_ids);
  // if right reverse
  if (lane.right_boundary_reverse()) {
    std::reverse(right_point_ids.begin(), right_point_ids.end());
  }
  // if left reverse
  if (lane.left_boundary_reverse()) {
    std::reverse(left_point_ids.begin(), left_point_ids.end());
  }
  LaneParser *lane_parser =
      new LaneParser({left_point_ids[0], left_point_ids.back()},
                     {right_point_ids[0], right_point_ids.back()});
  lane_relation_parser[lane.id()] = lane_parser;
}

std::string GetBoundaryIdByLaneId(const apollo::map_tool::EditorMap &editor_map,
                                  const std::string &lane_id, bool if_right) {
  apollo::map_tool::Lane lane;
  GetLaneByLaneId(editor_map, lane_id, &lane);
  if (if_right) {
    return lane.right_boundary_id();
  }
  return lane.left_boundary_id();
}

void GetBoundaryPointIdByLaneId(const apollo::map_tool::EditorMap &editor_map,
                                const std::string &lane_id,
                                std::vector<std::string> &point_ids,
                                const bool &if_right) {
  std::string boundary_id =
      GetBoundaryIdByLaneId(editor_map, lane_id, if_right);
  GetPointIdByBoundaryId(editor_map, boundary_id, point_ids);
}

bool GetLaneByLaneId(const apollo::map_tool::EditorMap &editor_map,
                     const std::string &lane_id, apollo::map_tool::Lane *lane) {
  for (const auto &iter : editor_map.lane()) {
    if (iter.id() == lane_id) {
      *lane = iter;
      return true;
    }
  }
  return false;
}

std::pair<std::vector<Vec2d>, std::vector<Vec2d>> GetLaneByLaneId(
    const apollo::map_tool::EditorMap &editor_map, const std::string &lane_id) {
  std::vector<std::string> left_point_ids;
  std::vector<std::string> right_point_ids;
  std::vector<Vec2d> left_points;
  std::vector<Vec2d> right_points;
  GetBoundaryPointIdByLaneId(editor_map, lane_id, left_point_ids, false);
  GetBoundaryPointIdByLaneId(editor_map, lane_id, right_point_ids, true);
  std::unordered_map<std::string, apollo::map_tool::Point> point_map;
  left_points = GetVec2dPoints(editor_map, left_point_ids);
  right_points = GetVec2dPoints(editor_map, right_point_ids);
  apollo::map_tool::Lane lane;
  GetLaneByLaneId(editor_map, lane_id, &lane);
  if (lane.left_boundary_reverse()) {
    std::reverse(left_points.begin(), left_points.end());
  }
  if (lane.right_boundary_reverse()) {
    std::reverse(right_points.begin(), right_points.end());
  }
  return {left_points, right_points};
}

void CalculateLaneSuccessiveRelationship(
    std::map<std::string, LaneParser *> &lane_relation_parser,
    const std::string &lane_id) {
  for (auto &lane_parser : lane_relation_parser) {
    std::string left_start_id = lane_parser.second->left_start_end_.first;
    std::string left_end_id = lane_parser.second->left_start_end_.second;
    std::string right_start_id = lane_parser.second->right_start_end_.first;
    std::string right_end_id = lane_parser.second->right_start_end_.second;
    if (lane_parser.first != lane_id) {
      if ((lane_relation_parser[lane_id]->right_start_end_.first ==
           right_end_id) &&
          (lane_relation_parser[lane_id]->left_start_end_.first ==
           left_end_id) &&
          right_start_id !=
              lane_relation_parser[lane_id]->right_start_end_.second &&
          left_start_id !=
              lane_relation_parser[lane_id]->left_start_end_.second) {
        lane_relation_parser[lane_id]->predecessor_id_.push_back(
            lane_parser.first);
        lane_parser.second->successor_id_.push_back(lane_id);
      }
    }
  }
}

void CalculateLaneNeighborRelationships(
    std::map<std::string, LaneParser *> &lane_relation_parser,
    const std::string &lane_id) {
  std::string curr_lane_left_start_id =
      lane_relation_parser[lane_id]->left_start_end_.first;
  std::string curr_lane_left_end_id =
      lane_relation_parser[lane_id]->left_start_end_.second;
  std::string curr_lane_right_start_id =
      lane_relation_parser[lane_id]->right_start_end_.first;
  std::string curr_lane_right_end_id =
      lane_relation_parser[lane_id]->right_start_end_.second;
  for (auto &lane_parser : lane_relation_parser) {
    std::string left_start_id = lane_parser.second->left_start_end_.first;
    std::string left_end_id = lane_parser.second->left_start_end_.second;
    std::string right_start_id = lane_parser.second->right_start_end_.first;
    std::string right_end_id = lane_parser.second->right_start_end_.second;
    if ((curr_lane_left_start_id == right_start_id) &&
        (curr_lane_left_end_id == right_end_id)) {
      lane_relation_parser[lane_id]->left_forward_neighbor_id_.push_back(
          lane_parser.first);
      lane_parser.second->right_forward_neighbor_id_.push_back(lane_id);
    } else if ((curr_lane_left_start_id == left_end_id) &&
               (curr_lane_left_end_id == left_start_id)) {
      lane_relation_parser[lane_id]->left_reverse_neighbor_id_.push_back(
          lane_parser.first);
    } else if ((curr_lane_right_start_id == right_end_id) &&
               (curr_lane_right_end_id == right_start_id)) {
      lane_relation_parser[lane_id]->right_reverse_neighbor_id_.push_back(
          lane_parser.first);
    }
  }
}

void CalculateLanesSuccessiveRelationships(
    std::map<std::string, LaneParser *> &lane_relation_parser) {
  for (const auto &curr_lane : lane_relation_parser) {
    CalculateLaneSuccessiveRelationship(lane_relation_parser, curr_lane.first);
  }
}

void CalculateLanesNeighborRelationships(
    std::map<std::string, LaneParser *> &lane_relation_parser) {
  for (const auto &curr_lane : lane_relation_parser) {
    CalculateLaneNeighborRelationships(lane_relation_parser, curr_lane.first);
  }
}

void CalculateCentralPointsOfLane(
    Vec2d &left_start_point, Vec2d &right_start_point, Vec2d &central_point,
    float step_length,
    const std::pair<std::vector<Vec2d>, std::vector<Vec2d>> &lane,
    LaneParser &lane_parser) {
  Vec2d next_left_start_point =
      GetNextPointFixedDistance(lane.first, left_start_point, step_length);
  Vec2d next_right_start_point =
      GetNextPointFixedDistance(lane.second, right_start_point, step_length);
  if (next_left_start_point.DistanceTo(lane.first[lane.first.size() - 1]) <
          step_length ||
      next_right_start_point.DistanceTo(lane.second[lane.second.size() - 1]) <
          step_length) {
    Vec2d left_end_point = lane.first[lane.first.size() - 1];
    Vec2d right_end_point = lane.second[lane.second.size() - 1];
    Vec2d last_center_point = lane_parser.central_points_.back();
    Vec2d final_center_points = (left_end_point + right_end_point) / 2;
    double left_end_width = final_center_points.DistanceTo(left_end_point);
    double right_end_width = final_center_points.DistanceTo(right_end_point);
    double final_center_point_s =
        final_center_points.DistanceTo(last_center_point);
    lane_parser.central_points_.push_back(final_center_points);
    lane_parser.left_cp_width_.push_back(left_end_width);
    lane_parser.right_cp_width_.push_back(right_end_width);
    lane_parser.cp_s_.push_back(final_center_point_s +
                                lane_parser.cp_s_.back());
    return;
  }
  Vec2d start_central_point = (left_start_point + right_start_point) / 2.0;
  Vec2d next_start_central_point =
      (next_left_start_point + next_right_start_point) / 2.0;
  LineSegment2d center_line(start_central_point, next_start_central_point);
  // Take the point where the center_line extension line is step distance from
  // the starting point.
  Vec2d end_center_point =
      GetNextPointFixedDistance(central_point, center_line, step_length);
  double center_left_width = 0;
  double center_right_width = 0;
  Vec2d new_left_start_point =
      GetPointDistanceToLane(lane.first, end_center_point, center_left_width);
  Vec2d new_right_start_point =
      GetPointDistanceToLane(lane.second, end_center_point, center_right_width);
  // add center point information.
  double center_point_s = start_central_point.DistanceTo(end_center_point);
  lane_parser.central_points_.push_back(end_center_point);
  lane_parser.left_cp_width_.push_back(center_left_width);
  lane_parser.right_cp_width_.push_back(center_right_width);
  lane_parser.cp_s_.push_back(center_point_s + lane_parser.cp_s_.back());
  // Iteration boundary conditions: The next point of any edge reaches the end
  // point of the lane boundary, and the center point is no longer calculated.
  // Enter iteration.
  CalculateCentralPointsOfLane(new_left_start_point, new_right_start_point,
                               end_center_point, step_length, lane,
                               lane_parser);
}

void CalculateCentralPointsOfCurveLane(
    const apollo::map_tool::EditorMap &editor_map,
    const Vec2d &central_start_point, const Vec2d &central_end_point,
    const apollo::map_tool::Lane &lane, LaneParser &lane_parser, double &step) {
  apollo::map_tool::Boundary left_boundary;
  apollo::map_tool::Boundary right_boundary;
  apollo::map_tool::Position central_lane_control_1;
  apollo::map_tool::Position central_lane_control_2;
  std::vector<apollo::map_tool::Position> central_interpolation_points;
  GetBoundaryByBoundaryId(editor_map, lane.left_boundary_id(), left_boundary);
  GetBoundaryByBoundaryId(editor_map, lane.right_boundary_id(), right_boundary);
  central_lane_control_1.set_x((left_boundary.controls_position(0).x() +
                                right_boundary.controls_position(0).x()) /
                               2.0);
  central_lane_control_1.set_y((left_boundary.controls_position(0).y() +
                                right_boundary.controls_position(0).y()) /
                               2.0);
  central_lane_control_2.set_x((left_boundary.controls_position(1).x() +
                                right_boundary.controls_position(1).x()) /
                               2.0);
  central_lane_control_2.set_y((left_boundary.controls_position(1).y() +
                                right_boundary.controls_position(1).y()) /
                               2.0);
  if (lane.right_boundary_reverse() && lane.left_boundary_reverse()) {
    central_lane_control_1.Swap(&central_lane_control_2);
  }
  apollo::map_tool::Position central_start_point_pb;
  apollo::map_tool::Position central_end_point_pb;
  central_start_point_pb.set_x(central_start_point.x());
  central_start_point_pb.set_y(central_start_point.y());
  central_end_point_pb.set_x(central_end_point.x());
  central_end_point_pb.set_y(central_end_point.y());
  GetInterpolationCurvePoints(central_start_point_pb, central_end_point_pb,
                              central_lane_control_1, central_lane_control_2,
                              &central_interpolation_points, step);
  Vec2d last_center_point = central_start_point;
  std::pair<std::vector<Vec2d>, std::vector<Vec2d>> lane_boundary;
  lane_boundary = GetLaneByLaneId(editor_map, lane.id());
  double center_left_width = 0;
  double center_right_width = 0;
  for (const auto &iter : central_interpolation_points) {
    Vec2d center_point((iter).x(), (iter).y());
    double center_point_s = center_point.DistanceTo(last_center_point);
    GetPointDistanceToLane(lane_boundary.first, center_point,
                           center_left_width);
    GetPointDistanceToLane(lane_boundary.second, center_point,
                           center_right_width);
    lane_parser.central_points_.push_back(center_point);
    lane_parser.left_cp_width_.push_back(center_left_width);
    lane_parser.right_cp_width_.push_back(center_right_width);
    lane_parser.cp_s_.push_back(center_point_s + lane_parser.cp_s_.back());
    last_center_point = center_point;
  }
  // add lastest center point
  double end_center_point_s = central_end_point.DistanceTo(last_center_point);
  GetPointDistanceToLane(lane_boundary.first, central_end_point,
                         center_left_width);
  GetPointDistanceToLane(lane_boundary.first, central_end_point,
                         center_right_width);
  lane_parser.central_points_.push_back(central_end_point);
  lane_parser.left_cp_width_.push_back(center_left_width);
  lane_parser.right_cp_width_.push_back(center_right_width);
  lane_parser.cp_s_.push_back(end_center_point_s + lane_parser.cp_s_.back());
}

void CalculateCentralPoints(
    const apollo::map_tool::EditorMap &editor_map,
    std::map<std::string, LaneParser *> &lane_relation_parser) {
  for (const auto &iter : editor_map.lane()) {
    std::pair<std::vector<Vec2d>, std::vector<Vec2d>> lane;
    lane = GetLaneByLaneId(editor_map, iter.id());
    Vec2d left_start_point = lane.first[0];
    Vec2d right_start_point = lane.second[0];
    Vec2d central_start_point = (left_start_point + right_start_point) / 2.0;
    lane_relation_parser[iter.id()]->central_points_.push_back(
        central_start_point);
    lane_relation_parser[iter.id()]->cp_s_.push_back(0);
    double center_left_width = 0;
    double center_right_width = 0;
    GetPointDistanceToLane(lane.first, central_start_point, center_left_width);
    GetPointDistanceToLane(lane.second, central_start_point,
                           center_right_width);
    lane_relation_parser[iter.id()]->left_cp_width_.push_back(
        center_left_width);
    lane_relation_parser[iter.id()]->right_cp_width_.push_back(
        center_right_width);
    if (iter.type() == apollo::map_tool::Lane_Type::Lane_Type_CURVE) {
      Vec2d central_end_point = (lane.first.back() + lane.second.back()) / 2.0;
      // calculate steps
      double step_left = lane.first.front().DistanceTo(lane.first.back()) /
                         FLAGS_curve_sampling_rate;
      double step_right = lane.second.front().DistanceTo(lane.second.back()) /
                          FLAGS_curve_sampling_rate;
      double step = step_left > step_right ? step_left : step_right;
      CalculateCentralPointsOfCurveLane(editor_map, central_start_point,
                                        central_end_point, iter,
                                        *lane_relation_parser[iter.id()], step);
    } else {
      CalculateCentralPointsOfLane(left_start_point, right_start_point,
                                   central_start_point, 1.0f, lane,
                                   *lane_relation_parser[iter.id()]);
    }
  }
}

void FillLaneNeighborSuccessorRelationships(
    apollo::hdmap::Lane *lane, apollo::map_tool::LaneParser *lane_parser) {
  for (const auto &successor_id : lane_parser->successor_id_) {
    lane->add_successor_id()->CopyFrom(
        GetMapElementId<apollo::hdmap::Lane>(successor_id));
  }
  for (const auto &predecessor_id : lane_parser->predecessor_id_) {
    lane->add_predecessor_id()->CopyFrom(
        GetMapElementId<apollo::hdmap::Lane>(predecessor_id));
  }
  for (const auto &left_neighbor_forward_id :
       lane_parser->left_forward_neighbor_id_) {
    lane->add_left_neighbor_forward_lane_id()->CopyFrom(
        GetMapElementId<apollo::hdmap::Lane>(left_neighbor_forward_id));
  }
  for (const auto &left_neighbor_reverse_id :
       lane_parser->left_reverse_neighbor_id_) {
    lane->add_left_neighbor_reverse_lane_id()->CopyFrom(
        GetMapElementId<apollo::hdmap::Lane>(left_neighbor_reverse_id));
  }
  for (const auto &right_neighbor_forward_id :
       lane_parser->right_forward_neighbor_id_) {
    lane->add_right_neighbor_forward_lane_id()->CopyFrom(
        GetMapElementId<apollo::hdmap::Lane>(right_neighbor_forward_id));
  }
  for (const auto &right_neighbor_reverse_id :
       lane_parser->right_reverse_neighbor_id_) {
    lane->add_right_neighbor_reverse_lane_id()->CopyFrom(
        GetMapElementId<apollo::hdmap::Lane>(right_neighbor_reverse_id));
  }
}

apollo::map_tool::Position CalculateCurveInterpolationPoint(
    const apollo::map_tool::Position &start,
    const apollo::map_tool::Position &end,
    const apollo::map_tool::Position &control_1,
    const apollo::map_tool::Position &control_2, double &t) {
  apollo::map_tool::Position interpolation_point;
  double x = pow(1 - t, 3) * start.x() + 3 * pow(1 - t, 2) * t * control_1.x() +
             3 * (1 - t) * pow(t, 2) * control_2.x() + pow(t, 3) * end.x();
  double y = pow(1 - t, 3) * start.y() + 3 * pow(1 - t, 2) * t * control_1.y() +
             3 * (1 - t) * pow(t, 2) * control_2.y() + pow(t, 3) * end.y();
  interpolation_point.set_x(x);
  interpolation_point.set_y(y);
  interpolation_point.set_z(0);
  return interpolation_point;
}

void GetInterpolationCurvePoints(
    const apollo::map_tool::Position &start,
    const apollo::map_tool::Position &end,
    const apollo::map_tool::Position &control_1,
    const apollo::map_tool::Position &control_2,
    std::vector<apollo::map_tool::Position> *points, double &step) {
  // Vec2d last_point = Vec2d(start.x(), start.y());
  // double accumulated_length = 0;
  // 计算一个插入速率 rate
  double rate = 1 / step;
  for (double t = rate; t < 1; t += rate) {
    apollo::map_tool::Position interpolation_point =
        CalculateCurveInterpolationPoint(start, end, control_1, control_2, t);
    // Vec2d curr_point(interpolation_point.x(), interpolation_point.y());
    // accumulated_length += curr_point.DistanceTo(last_point);
    // 曲线均匀取值逻辑, 暂时不用
    // if (accumulated_length >= FLAGS_curve_sampling_step_size) {
    //     points->push_back(interpolation_point);
    //     accumulated_length = 0;
    //     last_point = curr_point;
    // }
    points->push_back(interpolation_point);
  }
}

void AddInterpolationToCurveBoundary(
    const std::string &boundary_id,
    const std::vector<apollo::map_tool::Position> &interpolation_points,
    apollo::map_tool::EditorMap *editor_map) {
  for (auto &boundary : *editor_map->mutable_boundary()) {
    // 曲线的boundary必须只能有两个点, 起点和终点
    if (boundary.id() == boundary_id && boundary.point_id_size() == 2) {
      std::string start_point_id = boundary.point_id(0);
      std::string end_point_id = boundary.point_id(1);
      boundary.clear_point_id();
      boundary.add_point_id(start_point_id);
      int interpolation_index = 0;
      for (const auto &iter : interpolation_points) {
        // cp_ is the abbreviation of curve point.
        std::string id =
            "cp_" + boundary_id + "_" + std::to_string(interpolation_index);
        apollo::map_tool::Point *point = editor_map->add_point();
        point->mutable_position()->CopyFrom(iter);
        point->set_id(id);
        boundary.add_point_id(id);
        interpolation_index++;
      }
      boundary.add_point_id(end_point_id);
      break;
    }
  }
}

}  // namespace map_tool
}  // namespace apollo
