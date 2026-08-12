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
#include "modules/map_creator/map_tool/parser/road_parser.h"

#include <string>

namespace apollo {
namespace map_tool {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::Road;

void GenerateRoad(
        std::list<apollo::hdmap::Id>& ids,
        const apollo::hdmap::BoundaryEdge& left_road_edge,
        const apollo::hdmap::BoundaryEdge& right_road_edge,
        const apollo::hdmap::Id& road_id,
        Road* road) {
    // set_id: Lane -> Road
    road->mutable_id()->CopyFrom(road_id);
    road->mutable_id()->mutable_id()->replace(0, 4, "Road");
    // add type
    road->set_type(Road::UNKNOWN);
    // add section
    hdmap::RoadSection* section = road->add_section();
    section->mutable_id()->set_id("1");
    for (auto iter : ids) {
        section->add_lane_id()->CopyFrom(iter);
    }
    auto outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
    // copy left boundary
    auto left_edge = outer_polygon->add_edge();
    left_edge->CopyFrom(left_road_edge);
    // copy right boundary
    auto right_edge = outer_polygon->add_edge();
    right_edge->CopyFrom(right_road_edge);
}

bool AddRoads(apollo::hdmap::Map* map, apollo::map_tool::EditorMap& editor_map) {
    std::set<std::string> mapped_lane;
    std::map<std::string, apollo::hdmap::Lane> lane_map;
    std::map<std::string, apollo::map_tool::Lane> editor_lane_map;
    std::map<std::string, apollo::map_tool::Boundary> editor_boundary_map;
    std::map<std::string, apollo::map_tool::RoadBoundary> editor_roadboundary_map;
    std::map<std::string, apollo::map_tool::Point> editor_point_map;
    Vec2d map_offset(editor_map.basemap_center().x(), editor_map.basemap_center().y());
    for (auto& iter : map->lane()) {
        lane_map[iter.id().id()] = iter;
    }
    for (auto& iter : editor_map.lane()) {
        editor_lane_map[iter.id()] = iter;
    }
    for (auto& iter : editor_map.boundary()) {
        editor_boundary_map[iter.id()] = iter;
    }
    for (auto& iter : editor_map.road_boundary()) {
        editor_roadboundary_map[iter.id()] = iter;
    }
    for (auto& iter : editor_map.point()) {
        editor_point_map[iter.id()] = iter;
    }
    for (auto& lane : *map->mutable_lane()) {
        if (mapped_lane.count(lane.id().id()) > 0) {
            continue;
        }
        std::list<apollo::hdmap::Id> lane_ids;
        const apollo::hdmap::Lane* rightmost_lane = FindRightmostLane(lane_map, lane, mapped_lane, &lane_ids);
        const apollo::hdmap::Lane* leftmost_lane = FindLeftmostLane(lane_map, lane, mapped_lane, &lane_ids);
        lane_ids.erase(lane_ids.begin());
        std::string rightmost_lane_id = rightmost_lane->id().id();
        std::string leftmost_lane_id = leftmost_lane->id().id();
        rightmost_lane_id = rightmost_lane_id.substr(5);
        leftmost_lane_id = leftmost_lane_id.substr(5);

        std::string right_laneboundary_id = editor_lane_map[rightmost_lane_id].right_boundary_id();
        std::string left_laneboundary_id = editor_lane_map[leftmost_lane_id].left_boundary_id();
        auto left_lane_boundary = editor_boundary_map[left_laneboundary_id];
        auto right_lane_boundary = editor_boundary_map[right_laneboundary_id];
        std::vector<Vec2d> left_edge;
        std::vector<Vec2d> right_edge;
        apollo::hdmap::BoundaryEdge left_road_edge;
        apollo::hdmap::BoundaryEdge right_road_edge;
        left_road_edge.set_type(hdmap::BoundaryEdge::LEFT_BOUNDARY);
        right_road_edge.set_type(hdmap::BoundaryEdge::RIGHT_BOUNDARY);

        // left border
        if (left_lane_boundary.road_boundary_id_size() > 0) {
            auto left_road_boundary = editor_roadboundary_map[left_lane_boundary.road_boundary_id(0)];
            //  curve
            if (left_road_boundary.controls_position_size() > 0) {
                left_edge = GetRoadBoundary(editor_point_map, left_road_boundary, true);
            } else {
                left_edge = GetRoadBoundary(editor_point_map, left_road_boundary, false);
            }
            apollo::hdmap::Curve* left_curve = left_road_edge.mutable_curve();
            GetBoundaryCurve(left_edge, left_curve, map_offset);
        } else {
            for (auto& point_id : left_lane_boundary.point_id()) {
                auto position = editor_point_map[point_id].position();
                Vec2d point(position.x(), position.y());
                left_edge.push_back(point);
            }
            left_road_edge.mutable_curve()->CopyFrom(leftmost_lane->left_boundary().curve());
        }

        // right border
        if (right_lane_boundary.road_boundary_id_size() > 0) {
            auto right_road_boundary = editor_roadboundary_map[right_lane_boundary.road_boundary_id(0)];
            //  curve
            if (right_road_boundary.controls_position_size() > 0) {
                right_edge = GetRoadBoundary(editor_point_map, right_road_boundary, true);
            } else {
                right_edge = GetRoadBoundary(editor_point_map, right_road_boundary, false);
            }
            apollo::hdmap::Curve* right_curve = right_road_edge.mutable_curve();
            GetBoundaryCurve(right_edge, right_curve, map_offset);
        } else {
            for (auto& point_id : right_lane_boundary.point_id()) {
                auto position = editor_point_map[point_id].position();
                Vec2d point(position.x(), position.y());
                right_edge.push_back(point);
            }
            right_road_edge.mutable_curve()->CopyFrom(rightmost_lane->right_boundary().curve());
        }
        CalculateRoadSample(map, lane_ids, left_edge, right_edge, map_offset);
        apollo::hdmap::Id road_id(lane.id());
        GenerateRoad(lane_ids, left_road_edge, right_road_edge, road_id, map->add_road());
    }
    return true;
}

const apollo::hdmap::Lane* FindRightmostLane(
        std::map<std::string, apollo::hdmap::Lane>& lane_map,
        const apollo::hdmap::Lane& lane,
        std::set<std::string>& mapped_lane,
        std::list<apollo::hdmap::Id>* ids) {
    ids->push_back(lane.id());
    mapped_lane.insert(lane.id().id());
    if (lane.right_neighbor_forward_lane_id_size() <= 0) {
        return &lane;
    }
    for (auto iter : lane.right_neighbor_forward_lane_id()) {
        const apollo::hdmap::Lane* rightmost_lane = FindRightmostLane(lane_map, lane_map[iter.id()], mapped_lane, ids);
        if (rightmost_lane != nullptr) {
            return rightmost_lane;
        }
    }
    return nullptr;
}

const apollo::hdmap::Lane* FindLeftmostLane(
        std::map<std::string, apollo::hdmap::Lane>& lane_map,
        const apollo::hdmap::Lane& lane,
        std::set<std::string>& mapped_lane,
        std::list<apollo::hdmap::Id>* ids) {
    ids->push_back(lane.id());
    mapped_lane.insert(lane.id().id());
    if (lane.left_neighbor_forward_lane_id_size() <= 0) {
        return &lane;
    }
    for (auto iter : lane.left_neighbor_forward_lane_id()) {
        const apollo::hdmap::Lane* leftmost_lane = FindLeftmostLane(lane_map, lane_map[iter.id()], mapped_lane, ids);
        if (leftmost_lane != nullptr) {
            return leftmost_lane;
        }
    }
    return nullptr;
}

std::vector<Vec2d> GetRoadBoundary(
        std::map<std::string, apollo::map_tool::Point>& point_map,
        const apollo::map_tool::RoadBoundary& road_boundary,
        bool if_curve) {
    std::vector<apollo::common::math::Vec2d> boundary;
    if (if_curve) {
        apollo::map_tool::Position start = point_map[road_boundary.point_id(0)].position();
        apollo::map_tool::Position end = point_map[road_boundary.point_id(1)].position();
        std::vector<apollo::map_tool::Position> interpolation_points;
        double step = Vec2d(start.x(), start.y()).DistanceTo(Vec2d(end.x(), end.y())) / FLAGS_curve_sampling_rate;
        GetInterpolationCurvePoints(
                start,
                end,
                road_boundary.controls_position(0),
                road_boundary.controls_position(1),
                &interpolation_points,
                step);
        boundary.push_back(Vec2d(start.x(), start.y()));
        for (const auto& iter : interpolation_points) {
            boundary.push_back(Vec2d(iter.x(), iter.y()));
        }
        boundary.push_back(Vec2d(end.x(), end.y()));
        return boundary;
    }
    for (const auto& iter : road_boundary.point_id()) {
        apollo::map_tool::Point point = point_map[iter];
        boundary.push_back(Vec2d(point.position().x(), point.position().y()));
    }
    return boundary;
}

void CalculateRoadSample(
        apollo::hdmap::Map* map,
        std::list<apollo::hdmap::Id>& lane_ids,
        std::vector<Vec2d>& left_road_boundary,
        std::vector<Vec2d>& right_road_boundary,
        Vec2d& map_offset) {
    std::set<std::string> lane_ids_set;
    for (auto& id : lane_ids) {
        lane_ids_set.insert(id.id());
    }
    for (auto& lane : *map->mutable_lane()) {
        auto iter = lane_ids_set.find(lane.id().id());
        if (iter != lane_ids_set.end()) {
            for (auto& curve_segment : lane.central_curve().segment()) {
                if (curve_segment.has_line_segment()) {
                    auto& line_segment = curve_segment.line_segment();
                    double left_road_s = 0.0;
                    double right_road_s = 0.0;
                    Vec2d center_first_p(
                            line_segment.point(0).x() - map_offset.x(), line_segment.point(0).y() - map_offset.y());
                    double s = 0;
                    double left_width = 0, right_width = 0;
                    Vec2d last_center_p = center_first_p;
                    for (auto& point : line_segment.point()) {
                        Vec2d vec_p = Vec2d(point.x() - map_offset.x(), point.y() - map_offset.y());
                        double left_width = 0.0, right_width = 0.0;
                        GetPointDistanceToLane(left_road_boundary, vec_p, left_width);
                        GetPointDistanceToLane(right_road_boundary, vec_p, right_width);

                        s += vec_p.DistanceTo(last_center_p);
                        auto* left_road_sample = lane.add_left_road_sample();
                        left_road_sample->set_width(left_width);
                        left_road_sample->set_s(s);
                        auto* right_road_sample = lane.add_right_road_sample();
                        right_road_sample->set_width(right_width);
                        right_road_sample->set_s(s);
                        last_center_p = vec_p;
                    }
                }
            }
        }
    }
}

}  // namespace map_tool
}  // namespace apollo
