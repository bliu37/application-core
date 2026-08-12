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

#include "modules/map_creator/map_tool/parser/overlap_parser.h"

using apollo::common::math::LineSegment2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::map_tool::LaneParser;
namespace apollo {
namespace map_tool {
bool CalculateOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, LaneParser *> &lane_relation_parser) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> lane_overlap_ids;
    if (!CalculateLaneAndCrosswalkOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and crosswalk overlaps failed!";
        return false;
    }
    if (!CalculateLaneAndJunctionOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and Junction overlaps failed!";
        return false;
    }
    if (!CalculateLaneAndSpeedBumpOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and speed bump overlaps failed!";
        return false;
    }
    if (!CalculateLaneAndSignalOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and signal overlaps failed!";
        return false;
    }
    if (!CalculateLaneAndParkingSpaceOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and parking space overlaps failed!";
        return false;
    }
    if (!CalculateYieldSignAndLaneOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and yield sign overlaps failed!";
        return false;
    }
    if (!CalculateStopSignAndLaneOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and stop sign overlaps failed!";
        return false;
    }
    if (!CalculateAreaAndLaneOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and area overlaps failed!";
        return false;
    }
    if (!CalculateBarrierGateOverlaps(editor_map, hdmap, lane_relation_parser, &lane_overlap_ids)) {
        AERROR << "The calculation of lane and barrier gate failed!";
        return false;
    }
    if (!CalculateAreaOverlaps(editor_map, hdmap)) {
        AERROR << "The calculation of Driveable and UnDriveable overlaps failed!";
        return false;
    }
    if (!CalculateJunctionAndSignalOverlaps(editor_map, hdmap)) {
        AERROR << "The calculation of junction and signal overlaps failed!";
        return false;
    }
    AddLaneOverlapId(hdmap, lane_overlap_ids);
    return true;
}

bool CalculateLaneAndSpeedBumpOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> speed_bump_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &speed_bump : editor_map.speed_bump()) {
            std::vector<std::string> speed_bump_point_ids;
            std::vector<Vec2d> speed_bump_points;
            GetPointIdByBoundaryId(editor_map, speed_bump.boundary_id(), speed_bump_point_ids);
            speed_bump_points = GetVec2dPoints(editor_map, speed_bump_point_ids);
            LineSegment2d speed_bump_line(speed_bump_points.front(), speed_bump_points.back());
            if (IfOverlap(editor_map, lane, speed_bump_line)) {
                // The default width of the speed bump is 0.1m
                apollo::hdmap::Id speed_bump_id = GetMapElementId<apollo::hdmap::SpeedBump>(speed_bump.id());
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id overlap_id
                        = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + speed_bump_id.id());
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                // Lane overlap info
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                GetOverlapInfo(speed_bump_line, central_points, &lane_overlap_info);
                // Add overlap info
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *speed_bump_object = overlap->add_object();
                speed_bump_object->mutable_id()->CopyFrom(speed_bump_id);
                speed_bump_object->mutable_speed_bump_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                speed_bump_overlap_ids.operator[](speed_bump_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to speed bump.
    for (auto &speed_bump : *hdmap->mutable_speed_bump()) {
        auto iter = speed_bump_overlap_ids.find(speed_bump.id().id());
        if (iter != speed_bump_overlap_ids.end()) {
            for (const auto &id : speed_bump_overlap_ids.at(speed_bump.id().id())) {
                speed_bump.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateLaneAndCrosswalkOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> crosswalk_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &crosswalk : editor_map.crosswalk()) {
            apollo::common::math::Polygon2d crosswalk_polygon = GetPolygon2d(editor_map, crosswalk);
            apollo::common::math::Polygon2d lane_polygon = GetPolygon2d(editor_map, lane);
            apollo::common::math::Polygon2d overlap_polygon;
            if (IfOverlap(editor_map, lane, crosswalk_polygon)) {
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id crosswalk_id = GetMapElementId<apollo::hdmap::Crosswalk>(crosswalk.id());
                apollo::hdmap::Id overlap_id
                        = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + crosswalk_id.id());
                apollo::hdmap::Id region_overlap_id
                        = GetMapElementId<apollo::hdmap::RegionOverlapInfo>(lane_id.id() + crosswalk_id.id());
                // Calculation overlap info.
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                apollo::hdmap::CrosswalkOverlapInfo crosswalk_overlap_info;
                apollo::hdmap::RegionOverlapInfo region_overlap_info;
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                // Here we temporarily calculate the overlapping area of two polygonal convex hulls, which can be
                // optimized to calculate the concave and convex hulls.
                crosswalk_polygon.ComputeOverlap(lane_polygon, &overlap_polygon);
                GetRegionOverlapInfo(editor_map, overlap_polygon, region_overlap_id, &region_overlap_info);
                GetOverlapInfo(overlap_polygon, central_points, true, region_overlap_id, &lane_overlap_info);
                crosswalk_overlap_info.mutable_region_overlap_id()->CopyFrom(region_overlap_id);
                // Add overlap info.
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                overlap->add_region_overlap()->CopyFrom(region_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *crosswalk_object = overlap->add_object();
                crosswalk_object->mutable_id()->CopyFrom(crosswalk_id);
                crosswalk_object->mutable_crosswalk_overlap_info()->CopyFrom(crosswalk_overlap_info);
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                crosswalk_overlap_ids.operator[](crosswalk_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to crosswalk.
    for (auto &crosswalk : *hdmap->mutable_crosswalk()) {
        auto iter = crosswalk_overlap_ids.find(crosswalk.id().id());
        if (iter != crosswalk_overlap_ids.end()) {
            for (const auto &id : crosswalk_overlap_ids.at(crosswalk.id().id())) {
                crosswalk.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateLaneAndSignalOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> signal_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &signal : editor_map.traffic_signal()) {
            apollo::map_tool::StopLine stop_line;
            std::vector<std::string> stop_line_point_ids;
            std::vector<Vec2d> stop_line_points;
            if (!GetStopLineById(editor_map, signal.stop_line_id(), stop_line)) {
                return false;
            }
            GetPointIdByBoundaryId(editor_map, stop_line.boundary_id(), stop_line_point_ids);
            stop_line_points = GetVec2dPoints(editor_map, stop_line_point_ids);
            LineSegment2d stop_line_segment(stop_line_points.front(), stop_line_points.back());
            if (IfOverlap(editor_map, lane, stop_line_segment)) {
                apollo::hdmap::Id signal_id = GetMapElementId<apollo::hdmap::Signal>(signal.id());
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id overlap_id = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + signal_id.id());
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                // Lane overlap info.
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                GetOverlapInfo(stop_line_segment, central_points, &lane_overlap_info);
                // Add overlap info.
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *signal_object = overlap->add_object();
                signal_object->mutable_id()->CopyFrom(signal_id);
                signal_object->mutable_signal_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                signal_overlap_ids.operator[](signal_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to signal.
    for (auto &signal : *hdmap->mutable_signal()) {
        auto iter = signal_overlap_ids.find(signal.id().id());
        if (iter != signal_overlap_ids.end()) {
            for (const auto &id : signal_overlap_ids.at(signal.id().id())) {
                signal.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateLaneAndJunctionOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> junction_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &junction : editor_map.junction()) {
            apollo::common::math::Polygon2d junction_polygon = GetPolygon2d(editor_map, junction);
            apollo::common::math::Polygon2d lane_polygon = GetPolygon2d(editor_map, lane);
            apollo::common::math::Polygon2d overlap_polygon;
            if (IfOverlap(editor_map, lane, junction_polygon)) {
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id junction_id = GetMapElementId<apollo::hdmap::Junction>(junction.id());
                apollo::hdmap::Id overlap_id = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + junction_id.id());
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                junction_polygon.ComputeOverlap(lane_polygon, &overlap_polygon);
                GetOverlapInfo(overlap_polygon, central_points, false, apollo::hdmap::Id(), &lane_overlap_info);
                // Add overlap info
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *junction_object = overlap->add_object();
                junction_object->mutable_id()->CopyFrom(junction_id);
                junction_object->mutable_junction_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                junction_overlap_ids.operator[](junction_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to junction.
    for (auto &junction : *hdmap->mutable_junction()) {
        auto iter = junction_overlap_ids.find(junction.id().id());
        if (iter != junction_overlap_ids.end()) {
            for (const auto &id : junction_overlap_ids.at(junction.id().id())) {
                junction.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateLaneAndParkingSpaceOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> parking_space_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &parking_space : editor_map.parking_space()) {
            apollo::common::math::Polygon2d parking_space_polygon = GetPolygon2d(editor_map, parking_space);
            apollo::common::math::Polygon2d lane_polygon = GetPolygon2d(editor_map, lane);
            apollo::common::math::Polygon2d overlap_polygon;
            if (IfOverlap(editor_map, lane, parking_space_polygon, "parking_space")) {
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id parking_space_id = GetMapElementId<apollo::hdmap::ParkingSpace>(parking_space.id());
                apollo::hdmap::Id overlap_id
                        = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + parking_space_id.id());
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                double start_s = 0.0, end_s = 0.0;
                CalculateParkingSpaceReferenceLine(central_points, parking_space_polygon, start_s, end_s);
                lane_overlap_info.set_start_s(start_s);
                lane_overlap_info.set_end_s(end_s);
                lane_overlap_info.set_is_merge(false);
                // Add overlap info
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *parking_space_object = overlap->add_object();
                parking_space_object->mutable_id()->CopyFrom(parking_space_id);
                parking_space_object->mutable_parking_space_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                parking_space_overlap_ids.operator[](parking_space_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to speed bump.
    for (auto &parking_space : *hdmap->mutable_parking_space()) {
        auto iter = parking_space_overlap_ids.find(parking_space.id().id());
        if (iter != parking_space_overlap_ids.end()) {
            for (const auto &id : parking_space_overlap_ids.at(parking_space.id().id())) {
                parking_space.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateBarrierGateOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    // Apollo 8.0 hdmap does not define BarrierGate; no overlaps generated.
    (void)editor_map;
    (void)hdmap;
    (void)lane_relation_parser;
    (void)lane_overlap_ids;
    return true;
}

bool CalculateYieldSignAndLaneOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> yield_sign_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &yield_sign : editor_map.yield_sign()) {
            apollo::map_tool::StopLine stop_line;
            std::vector<std::string> stop_line_point_ids;
            std::vector<Vec2d> stop_line_points;
            if (!GetStopLineById(editor_map, yield_sign.stop_line_id(), stop_line)) {
                return false;
            }
            GetPointIdByBoundaryId(editor_map, stop_line.boundary_id(), stop_line_point_ids);
            stop_line_points = GetVec2dPoints(editor_map, stop_line_point_ids);
            LineSegment2d stop_line_segment(stop_line_points.front(), stop_line_points.back());
            if (IfOverlap(editor_map, lane, stop_line_segment)) {
                apollo::hdmap::Id yield_id = GetMapElementId<apollo::hdmap::YieldSign>(yield_sign.id());
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id overlap_id = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + yield_id.id());
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                // Lane overlap info.
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                GetOverlapInfo(stop_line_segment, central_points, &lane_overlap_info);
                // Add overlap info.
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *yield_sign_object = overlap->add_object();
                yield_sign_object->mutable_id()->CopyFrom(yield_id);
                yield_sign_object->mutable_yield_sign_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                yield_sign_overlap_ids.operator[](yield_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to yield sign.
    for (auto &yield_sign : *hdmap->mutable_yield()) {
        auto iter = yield_sign_overlap_ids.find(yield_sign.id().id());
        if (iter != yield_sign_overlap_ids.end()) {
            for (const auto &id : yield_sign_overlap_ids.at(yield_sign.id().id())) {
                yield_sign.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateStopSignAndLaneOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> stop_sign_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &stop_sign : editor_map.stop_sign()) {
            apollo::map_tool::StopLine stop_line;
            std::vector<std::string> stop_line_point_ids;
            std::vector<Vec2d> stop_line_points;
            if (!GetStopLineById(editor_map, stop_sign.stop_line_id(), stop_line)) {
                return false;
            }
            GetPointIdByBoundaryId(editor_map, stop_line.boundary_id(), stop_line_point_ids);
            stop_line_points = GetVec2dPoints(editor_map, stop_line_point_ids);
            LineSegment2d stop_line_segment(stop_line_points.front(), stop_line_points.back());
            if (IfOverlap(editor_map, lane, stop_line_segment)) {
                apollo::hdmap::Id stop_sign_id = GetMapElementId<apollo::hdmap::StopSign>(stop_sign.id());
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id overlap_id
                        = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + stop_sign_id.id());
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                // Lane overlap info.
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                GetOverlapInfo(stop_line_segment, central_points, &lane_overlap_info);
                // Add overlap info.
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *stop_sign_object = overlap->add_object();
                stop_sign_object->mutable_id()->CopyFrom(stop_sign_id);
                stop_sign_object->mutable_stop_sign_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                stop_sign_overlap_ids.operator[](stop_sign_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to yield sign.
    for (auto &stop_sign : *hdmap->mutable_stop_sign()) {
        auto iter = stop_sign_overlap_ids.find(stop_sign.id().id());
        if (iter != stop_sign_overlap_ids.end()) {
            for (const auto &id : stop_sign_overlap_ids.at(stop_sign.id().id())) {
                stop_sign.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateJunctionAndSignalOverlaps(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *hdmap) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> signal_overlap_ids;
    for (const auto &junction : editor_map.junction()) {
        std::map<std::string, std::vector<apollo::hdmap::Id>> junction_overlap_ids;
        for (const auto &signal : editor_map.traffic_signal()) {
            apollo::map_tool::StopLine stop_line;
            std::vector<std::string> stop_line_point_ids;
            std::vector<Vec2d> stop_line_points;
            if (!GetStopLineById(editor_map, signal.stop_line_id(), stop_line)) {
                return false;
            }
            GetPointIdByBoundaryId(editor_map, stop_line.boundary_id(), stop_line_point_ids);
            stop_line_points = GetVec2dPoints(editor_map, stop_line_point_ids);
            LineSegment2d stop_line_segment(stop_line_points.front(), stop_line_points.back());
            apollo::common::math::Polygon2d junction_polygon = GetPolygon2d(editor_map, junction);
            if (IfOverlap(junction_polygon, stop_line_segment)) {
                apollo::hdmap::Id signal_id = GetMapElementId<apollo::hdmap::Lane>(signal.id());
                apollo::hdmap::Id junction_id = GetMapElementId<apollo::hdmap::Junction>(junction.id());
                apollo::hdmap::Id overlap_id
                        = GetMapElementId<apollo::hdmap::Overlap>(junction_id.id() + signal_id.id());
                // Add overlap info
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *junction_object = overlap->add_object();
                junction_object->mutable_id()->CopyFrom(junction_id);
                junction_object->mutable_lane_overlap_info();
                apollo::hdmap::ObjectOverlapInfo *singal_object = overlap->add_object();
                singal_object->mutable_id()->CopyFrom(signal_id);
                singal_object->mutable_signal_overlap_info();
                junction_overlap_ids.operator[](junction_id.id()).push_back(overlap_id);
                signal_overlap_ids.operator[](signal_id.id()).push_back(overlap_id);
            }
        }
        // Add overlap ids to junction.
        for (auto &junction : *hdmap->mutable_junction()) {
            auto iter = junction_overlap_ids.find(junction.id().id());
            if (iter != junction_overlap_ids.end()) {
                for (const auto &id : junction_overlap_ids.at(junction.id().id())) {
                    junction.add_overlap_id()->CopyFrom(id);
                }
            }
        }
    }
    // Add overlap ids to signal.
    for (auto &signal : *hdmap->mutable_signal()) {
        auto iter = signal_overlap_ids.find(signal.id().id());
        if (iter != signal_overlap_ids.end()) {
            for (const auto &id : signal_overlap_ids.at(signal.id().id())) {
                signal.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateAreaAndLaneOverlaps(
        const apollo::map_tool::EditorMap &editor_map,
        apollo::hdmap::Map *hdmap,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        std::map<std::string, std::vector<apollo::hdmap::Id>> *lane_overlap_ids) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> area_overlap_ids;
    for (const auto &lane : editor_map.lane()) {
        for (const auto &area : editor_map.area()) {
            apollo::common::math::Polygon2d area_polygon = GetPolygon2d(editor_map, area);
            apollo::common::math::Polygon2d lane_polygon = GetPolygon2d(editor_map, lane);
            apollo::common::math::Polygon2d overlap_polygon;
            if (IfOverlap(editor_map, lane, area_polygon)) {
                apollo::hdmap::Id lane_id = GetMapElementId<apollo::hdmap::Lane>(lane.id());
                apollo::hdmap::Id area_id = GetMapElementId<apollo::hdmap::ClearArea>(area.id());
                apollo::hdmap::Id overlap_id = GetMapElementId<apollo::hdmap::Overlap>(lane_id.id() + area_id.id());
                apollo::hdmap::LaneOverlapInfo lane_overlap_info;
                std::vector<Vec2d> central_points = lane_relation_parser[lane.id()]->central_points_;
                area_polygon.ComputeOverlap(lane_polygon, &overlap_polygon);
                GetOverlapInfo(overlap_polygon, central_points, false, apollo::hdmap::Id(), &lane_overlap_info);
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *lane_object = overlap->add_object();
                lane_object->mutable_id()->CopyFrom(lane_id);
                lane_object->mutable_lane_overlap_info()->CopyFrom(lane_overlap_info);
                apollo::hdmap::ObjectOverlapInfo *area_object = overlap->add_object();
                area_object->mutable_id()->CopyFrom(area_id);
                area_object->mutable_clear_area_overlap_info();
                lane_overlap_ids->operator[](lane_id.id()).push_back(overlap_id);
                area_overlap_ids.operator[](area_id.id()).push_back(overlap_id);
            }
        }
    }
    // Add overlap ids to clear_area
    for (auto &area : *hdmap->mutable_clear_area()) {
        auto iter = area_overlap_ids.find(area.id().id());
        if (iter != area_overlap_ids.end()) {
            for (const auto &id : area_overlap_ids.at(area.id().id())) {
                area.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool CalculateAreaOverlaps(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *hdmap) {
    std::map<std::string, std::vector<apollo::hdmap::Id>> area_undriveable_overlap_ids;
    for (const auto &area_driveable : editor_map.area()) {
        if (area_driveable.type() == Area_Type::Area_Type_UNDRIVEABLE) {
            continue;
        }
        std::map<std::string, std::vector<apollo::hdmap::Id>> area_driveable_overlap_ids;
        for (const auto &area_undriveable : editor_map.area()) {
            if (area_undriveable.type() != Area_Type::Area_Type_UNDRIVEABLE) {
                continue;
            }
            apollo::common::math::Polygon2d area_driveable_polygon = GetPolygon2d(editor_map, area_driveable);
            apollo::common::math::Polygon2d area_undriveable_polygon = GetPolygon2d(editor_map, area_undriveable);
            apollo::common::math::Polygon2d overlap_polygon;
            if (IfOverlap(area_driveable_polygon, area_undriveable_polygon)) {
                apollo::hdmap::Id area_driveable_id = GetMapElementId<apollo::hdmap::ClearArea>(area_driveable.id());
                apollo::hdmap::Id area_undriveable_id = GetMapElementId<apollo::hdmap::ClearArea>(area_undriveable.id());
                apollo::hdmap::Id overlap_id
                        = GetMapElementId<apollo::hdmap::Overlap>(area_driveable_id.id() + area_undriveable_id.id());
                apollo::hdmap::Overlap *overlap = hdmap->add_overlap();
                overlap->mutable_id()->CopyFrom(overlap_id);
                apollo::hdmap::ObjectOverlapInfo *area_driveable_object = overlap->add_object();
                area_driveable_object->mutable_id()->CopyFrom(area_driveable_id);
                area_driveable_object->mutable_clear_area_overlap_info();
                apollo::hdmap::ObjectOverlapInfo *area_undriveable_object = overlap->add_object();
                area_undriveable_object->mutable_id()->CopyFrom(area_undriveable_id);
                area_undriveable_object->mutable_clear_area_overlap_info();
                area_driveable_overlap_ids.operator[](area_driveable_id.id()).push_back(overlap_id);
                area_undriveable_overlap_ids.operator[](area_undriveable_id.id()).push_back(overlap_id);
            }
        }
        for (auto &area : *hdmap->mutable_clear_area()) {
            auto iter = area_driveable_overlap_ids.find(area.id().id());
            if (iter != area_driveable_overlap_ids.end()) {
                for (const auto &id : area_driveable_overlap_ids.at(area.id().id())) {
                    area.add_overlap_id()->CopyFrom(id);
                }
            }
        }
    }
    // Add overlap ids to area
    for (auto &area : *hdmap->mutable_clear_area()) {
        auto iter = area_undriveable_overlap_ids.find(area.id().id());
        if (iter != area_undriveable_overlap_ids.end()) {
            for (const auto &id : area_undriveable_overlap_ids.at(area.id().id())) {
                area.add_overlap_id()->CopyFrom(id);
            }
        }
    }
    return true;
}

bool IfOverlap(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        const apollo::common::math::Polygon2d &polygon) {
    std::vector<LineSegment2d> left_boundary;
    std::vector<LineSegment2d> right_boundary;
    std::vector<Vec2d> left_intersects;
    std::vector<Vec2d> right_intersects;
    std::pair<std::vector<Vec2d>, std::vector<Vec2d>> lane_boundary;
    lane_boundary = GetLaneByLaneId(editor_map, lane.id());
    left_boundary = GetLineSegment2ds(lane_boundary.first);
    right_boundary = GetLineSegment2ds(lane_boundary.second);
    bool if_left_boundary_intersect = false;
    bool if_right_boundary_intersect = false;
    for (const auto &iter : left_boundary) {
        if (polygon.HasOverlap(iter)) {
            if_left_boundary_intersect = true;
        }
    }
    for (const auto &iter : right_boundary) {
        if (polygon.HasOverlap(iter)) {
            if_right_boundary_intersect = true;
        }
    }
    if (if_left_boundary_intersect && if_right_boundary_intersect) {
        return true;
    } else {
        bool if_left_has_near_line = false;
        bool if_right_has_near_line = false;
        for (const auto &iter : left_boundary) {
            if (polygon.DistanceTo(iter) <= FLAGS_overlap_error_range) {
                if_left_has_near_line = true;
            }
        }
        for (const auto &iter : right_boundary) {
            if (polygon.DistanceTo(iter) <= FLAGS_overlap_error_range) {
                if_right_has_near_line = true;
            }
        }
        if (if_left_has_near_line && if_right_has_near_line) {
            return true;
        }
    }
    return false;
}

bool IfOverlap(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        const apollo::common::math::Polygon2d &polygon,
        std::string type) {
    if (type == "parking_space") {
        std::vector<LineSegment2d> right_boundary;
        std::vector<LineSegment2d> left_boundary;
        std::vector<Vec2d> right_intersects;
        std::pair<std::vector<Vec2d>, std::vector<Vec2d>> lane_boundary;
        lane_boundary = GetLaneByLaneId(editor_map, lane.id());
        right_boundary = GetLineSegment2ds(lane_boundary.second);
        left_boundary = GetLineSegment2ds(lane_boundary.first);
        for (const auto &iter : right_boundary) {
            if (polygon.HasOverlap(iter)
                || (polygon.DistanceTo(iter) <= FLAGS_default_associated_distance_parking_space_lane)) {
                return true;
            }
        }
        for (const auto &iter : left_boundary) {
            if (polygon.HasOverlap(iter)
                || (polygon.DistanceTo(iter) <= FLAGS_default_associated_distance_parking_space_lane)) {
                return true;
            }
        }
    }
    return false;
}

bool IfOverlap(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::map_tool::Lane &lane,
        const LineSegment2d &segment) {
    std::pair<std::vector<Vec2d>, std::vector<Vec2d>> lane_boundary;
    std::vector<LineSegment2d> left_boundary;
    std::vector<LineSegment2d> right_boundary;
    lane_boundary = GetLaneByLaneId(editor_map, lane.id());
    left_boundary = GetLineSegment2ds(lane_boundary.first);
    right_boundary = GetLineSegment2ds(lane_boundary.second);
    Vec2d left_intersect;
    Vec2d right_intersect;
    if (GetIntersect(segment, left_boundary, &left_intersect)
        && GetIntersect(segment, right_boundary, &right_intersect)) {
        return true;
    } else {
        bool if_left_has_near_line = false;
        bool if_right_has_near_line = false;
        for (const auto &iter : left_boundary) {
            if ((iter.DistanceTo(segment.start()) <= FLAGS_overlap_error_range)
                != (iter.DistanceTo(segment.end()) <= FLAGS_overlap_error_range)) {
                if_left_has_near_line = true;
            }
        }
        for (const auto &iter : right_boundary) {
            if ((iter.DistanceTo(segment.start()) <= FLAGS_overlap_error_range)
                != (iter.DistanceTo(segment.end()) <= FLAGS_overlap_error_range)) {
                if_right_has_near_line = true;
            }
        }
        if (if_left_has_near_line && if_right_has_near_line) {
            return true;
        }
    }
    return false;
}

bool IfOverlap(const Polygon2d &polygon, const LineSegment2d &segment) {
    if (polygon.HasOverlap(segment)) {
        return true;
    }
    return false;
}

bool IfOverlap(const Polygon2d &polygon_1, const Polygon2d &polygon_2) {
    if (polygon_1.HasOverlap(polygon_2)) {
        return true;
    }
    return false;
}

void GetOverlapInfo(
        const apollo::common::math::Polygon2d &overlap_polygon,
        const std::vector<Vec2d> &central_points,
        bool if_need_region,
        const apollo::hdmap::Id region_overlap_id,
        apollo::hdmap::LaneOverlapInfo *lane_overlap_info) {
    if (if_need_region) {
        lane_overlap_info->mutable_region_overlap_id()->CopyFrom(region_overlap_id);
    }
    Vec2d start_point;
    Vec2d end_point;
    double start_s = 0;
    double end_s = 0;
    std::vector<LineSegment2d> central_line;
    std::vector<Vec2d> intersects;
    central_line = GetLineSegment2ds(central_points);
    if (GetIntersect(overlap_polygon, central_line, &intersects)) {
        start_s = GetDistanceTo(central_line, intersects.front());
        end_s = GetDistanceTo(central_line, intersects.back());
    } else if (intersects.size() == 1) {
        // If the starting point of the center line is in the overlap area, else the ending point of the center line is
        // in the overlap area.
        if (overlap_polygon.IsPointIn(central_points.front())
            || overlap_polygon.IsPointOnBoundary(central_points.front())) {
            start_s = 0;
            end_s = GetDistanceTo(central_line, intersects.front());
        } else {
            start_s = GetDistanceTo(central_line, intersects.front());
            end_s = GetDistanceTo(central_line, central_points.back());
        }
    };
    lane_overlap_info->set_start_s(start_s);
    lane_overlap_info->set_end_s(end_s);
    lane_overlap_info->set_is_merge(false);
}

void GetOverlapInfo(
        const apollo::common::math::LineSegment2d &segment,
        const std::vector<Vec2d> &central_points,
        apollo::hdmap::LaneOverlapInfo *lane_overlap_info) {
    Vec2d center_intersect;
    std::vector<LineSegment2d> central_line;
    central_line = GetLineSegment2ds(central_points);
    GetIntersect(segment, central_line, &center_intersect);
    double start_s = GetDistanceTo(central_line, center_intersect);
    double end_s = start_s + 0.1;
    lane_overlap_info->set_start_s(start_s);
    lane_overlap_info->set_end_s(end_s);
    lane_overlap_info->set_is_merge(false);
}

void GetRegionOverlapInfo(
        const apollo::map_tool::EditorMap &editor_map,
        const apollo::common::math::Polygon2d &overlap_polygon,
        apollo::hdmap::Id &region_overlap_id,
        apollo::hdmap::RegionOverlapInfo *region_overlap_info) {
    apollo::hdmap::Polygon overlap_polygon_pb = GetPolygonByPoints(editor_map, overlap_polygon.points());
    region_overlap_info->mutable_id()->CopyFrom(region_overlap_id);
    region_overlap_info->add_polygon()->CopyFrom(overlap_polygon_pb);
}

void AddLaneOverlapId(apollo::hdmap::Map *hdmap, std::map<std::string, std::vector<apollo::hdmap::Id>> &ids) {
    for (auto &lane : *hdmap->mutable_lane()) {
        auto iter = ids.find(lane.id().id());
        if (iter != ids.end()) {
            for (const auto &id : ids[lane.id().id()]) {
                lane.add_overlap_id()->CopyFrom(id);
            }
        }
    }
}

void CalculateParkingSpaceReferenceLine(
        const std::vector<Vec2d> &central_points,
        apollo::common::math::Polygon2d parking_space,
        double &start_s,
        double &end_s) {
    Vec2d max_point;
    Vec2d min_point;
    double distance = 0.0;
    double max_distance = 0.0, min_distance = std::numeric_limits<double>::max();
    for (auto &point : parking_space.points()) {
        Vec2d vertical_foot = GetPointDistanceToLane(central_points, point, distance);
        double distance = central_points[0].DistanceTo(vertical_foot);
        if (distance > max_distance) {
            max_distance = distance;
            max_point = vertical_foot;
        }
        if (distance < min_distance) {
            min_distance = distance;
            min_point = vertical_foot;
        }
    }
    std::vector<apollo::common::math::LineSegment2d> center_line_segment = GetLineSegment2ds(central_points);
    start_s = GetDistanceTo(center_line_segment, min_point);
    end_s = GetDistanceTo(center_line_segment, max_point);
}

}  // namespace map_tool
}  // namespace apollo
