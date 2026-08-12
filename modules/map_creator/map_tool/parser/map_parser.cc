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
#include "modules/map_creator/map_tool/parser/map_parser.h"

#include <fstream>
namespace apollo {
namespace map_tool {

#define GEO_INFO_FILE "tiles.json"

using Json = nlohmann::json;
using apollo::common::util::DownsampleByAngle;
using apollo::common::util::DownsampleByDistance;

namespace {

std::string ResolveMapToolConfPath(const std::string &configured_path) {
    if (apollo::cyber::common::PathExists(configured_path)) {
        return configured_path;
    }

    const std::size_t file_name_pos = configured_path.find_last_of('/');
    const std::string file_name = file_name_pos == std::string::npos
            ? configured_path
            : configured_path.substr(file_name_pos + 1);
    const char *const prefixes[] = {
            "/apollo_workspace/modules/map_creator/map_tool/conf/",
            "/opt/apollo/neo/share/modules/map_creator/map_tool/conf/",
            "/apollo/modules/map_creator/map_tool/conf/",
    };

    for (const char *prefix : prefixes) {
        const std::string candidate = std::string(prefix) + file_name;
        if (candidate == configured_path) {
            continue;
        }
        if (apollo::cyber::common::PathExists(candidate)) {
            AWARN << "Map tool config path does not exist: " << configured_path
                  << ", using fallback: " << candidate;
            return candidate;
        }
    }

    return configured_path;
}

}  // namespace

bool CovertLanes(
        const apollo::map_tool::EditorMap &editor_map,
        std::map<std::string, apollo::map_tool::LaneParser *> &lane_relation_parser,
        apollo::hdmap::Map *map) {
    if (!CalculateLaneRelations(editor_map, lane_relation_parser)) {
        return false;
    }
    // calcuate center points
    CalculateCentralPoints(editor_map, lane_relation_parser);
    for (const auto &iter : editor_map.lane()) {
        apollo::hdmap::Lane *lane = map->add_lane();
        apollo::hdmap::LaneBoundary left_boundary;
        apollo::hdmap::LaneBoundary right_boundary;
        apollo::hdmap::Lane::LaneTurn lane_turn;
        apollo::hdmap::Lane::LaneType lane_type;
        if (!GetLaneTurnType(iter.attr().direction(), lane_turn)) {
            return false;
        }
        if (!GetLaneType(iter.attr().lane_type(), lane_type)) {
            return false;
        }
        FillLaneNeighborSuccessorRelationships(lane, lane_relation_parser[iter.id()]);
        for (std::size_t i = 0; i < lane_relation_parser[iter.id()]->cp_s_.size(); i++) {
            apollo::hdmap::LaneSampleAssociation left_sample;
            apollo::hdmap::LaneSampleAssociation right_sample;
            left_sample.set_s(lane_relation_parser[iter.id()]->cp_s_[i]);
            right_sample.set_s(lane_relation_parser[iter.id()]->cp_s_[i]);
            auto left_width = lane_relation_parser[iter.id()]->left_cp_width_[i];
            auto right_width = lane_relation_parser[iter.id()]->right_cp_width_[i];
            left_sample.set_width(left_width);
            right_sample.set_width(right_width);
            lane->add_left_sample()->CopyFrom(left_sample);
            lane->add_right_sample()->CopyFrom(right_sample);
        }
        GetLaneBoundary(editor_map, lane_relation_parser, iter.id(), iter.type(), lane);
        GetLaneCentralCurve(editor_map, lane_relation_parser, iter.id(), lane);
        lane->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::Lane>(iter.id()));
        lane->set_turn(lane_turn);
        lane->set_type(lane_type);
        lane->set_direction(apollo::hdmap::Lane::FORWARD);
        lane->set_length(GetLaneLength(lane_relation_parser, iter.id()));
        lane->set_speed_limit(iter.attr().speed() / 3.6);
    }
    return true;
}

bool EditorMapParser(apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    std::map<std::string, apollo::map_tool::LaneParser *> lane_relation_parser;
    ConvertCurveTolane(&editor_map, &lane_relation_parser);
    AddReverseLaneToEditorMap(&editor_map);
    if (!CovertHeader(editor_map, map)) {
        AERROR << "Failed to convert header to HD map.";
        return false;
    }
    if (!CovertLanes(editor_map, lane_relation_parser, map)) {
        AERROR << "Failed to convert lane of HD map.";
        return false;
    }
    if (!CovertJunctions(editor_map, map)) {
        AERROR << "Failed to convert junctions of HD map.";
        return false;
    }
    if (!CovertSpeedBumps(editor_map, map)) {
        AERROR << "Failed to covert speedbumps of HD map.";
        return false;
    }
    if (!CovertCrosswalks(editor_map, map)) {
        AERROR << "Failed to covert crosswalks of HD map.";
        return false;
    }
    if (!CovertParkingSpaces(editor_map, map)) {
        AERROR << "Failed to covert parking spaces of HD map.";
        return false;
    }
    if (!CovertTrafficSignals(editor_map, map)) {
        AERROR << "Failed to covert traffic signals of HD map.";
        return false;
    }
    if (!CovertStopSigns(editor_map, map)) {
        AERROR << "Failed to covert stop signs of HD map.";
        return false;
    }
    if (!CovertYieldSigns(editor_map, map)) {
        AERROR << "Failed to covert yield signs of HD map.";
        return false;
    }
    if (!CovertArea(editor_map, map)) {
        AERROR << "Failed to covert area of HD map.";
        return false;
    }
    // BarrierGate is not supported in Apollo 8.0 hdmap. Skip conversion.
    if (!CovertBarrierGate(editor_map, map)) {
        AERROR << "Failed to covert barrier gate of HD map.";
        return false;
    }
    if (!CalculateOverlaps(editor_map, map, lane_relation_parser)) {
        AERROR << "Failed to calculate overlaps of HD map.";
        return false;
    }
    if (!AddRoads(map, editor_map)) {
        AERROR << "Failed to add road into HD map.";
        return false;
    }
    return true;
}

bool EditorMapParser(const Json &json, apollo::hdmap::Map *map) {
    apollo::map_tool::EditorMap editor_map;
    if (!apollo::map_tool::GetProtoFromJson(json, &editor_map)) {
        AERROR << "Failed to save editor map from json.";
        return false;
    }
    if (!EditorMapParser(editor_map, map)) {
        AERROR << "Failed to parse editor map into hdmap.";
        return false;
    }
    return true;
}

void AddReverseLaneToEditorMap(apollo::map_tool::EditorMap *editor_map) {
    apollo::map_tool::EditorMap editor_map_temp;
    editor_map_temp.CopyFrom(*editor_map);
    for (auto lane : editor_map_temp.lane()) {
        if (lane.attr().prossible_driving_direction() == apollo::map_tool::LaneAttribute::RELATIVEDIRECTION) {
            apollo::map_tool::Lane *reverse_lane = editor_map->add_lane();
            reverse_lane->CopyFrom(lane);
            reverse_lane->clear_id();
            reverse_lane->set_id(lane.id() + "_reverse");
            reverse_lane->clear_left_boundary_id();
            reverse_lane->clear_right_boundary_id();
            reverse_lane->clear_left_boundary_reverse();
            reverse_lane->clear_right_boundary_reverse();
            reverse_lane->set_left_boundary_id(lane.right_boundary_id());
            reverse_lane->set_right_boundary_id(lane.left_boundary_id());
            reverse_lane->set_left_boundary_reverse(!lane.left_boundary_reverse());
            reverse_lane->set_right_boundary_reverse(!lane.right_boundary_reverse());
        }
    }
}

void DownsampleCurve(apollo::hdmap::Curve *curve) {
    auto *line_segment = curve->mutable_segment(0)->mutable_line_segment();
    std::vector<apollo::common::PointENU> points(line_segment->point().begin(), line_segment->point().end());
    line_segment->clear_point();

    // NOTE: this not the most efficient implementation, but since this map tool
    // is only run once for each, we can probably live with that.

    // Downsample points by angle then by distance.
    auto sampled_indices = DownsampleByAngle(points, 1. / 180 * M_PI);
    std::vector<apollo::common::PointENU> downsampled_points;
    for (const size_t index : sampled_indices) {
        downsampled_points.push_back(points[index]);
    }

    sampled_indices = DownsampleByDistance(downsampled_points, 5, 1);

    for (const size_t index : sampled_indices) {
        *line_segment->add_point() = downsampled_points[index];
    }
    size_t new_size = line_segment->point_size();
    CHECK_GT(new_size, 1U);

    AINFO << "Lane curve downsampled from " << points.size() << " points to " << new_size << " points.";
}

void GenerateSimMap(const std::string file_name, apollo::hdmap::Map *map) {
    for (int i = 0; i < map->lane_size(); ++i) {
        auto *lane = map->mutable_lane(i);
        AINFO << "Downsampling lane " << lane->id().id();
        DownsampleCurve(lane->mutable_central_curve());
        DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
        DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
    }
    SaveSimMap(file_name, *map);
}

void SaveSimMap(const std::string file_name, const apollo::hdmap::Map &map_pb) {
    std::ofstream map_txt_file(file_name + "/sim_map.txt");
    map_txt_file << map_pb.DebugString();
    map_txt_file.close();

    std::ofstream map_bin_file(file_name + "/sim_map.bin");
    std::string map_str;
    map_pb.SerializeToString(&map_str);
    map_bin_file << map_str;
    map_bin_file.close();
}

bool GenerateRoutingMap(const std::string &base_map_path) {
    std::string base_map_file_path = base_map_path + "/base_map.bin";
    std::string routing_map_file_path = base_map_path + "/routing_map.bin";
    apollo::routing::RoutingConfig conf;
    if (!apollo::cyber::common::GetProtoFromFile(FLAGS_routing_map_config_path, &conf)) {
        AERROR << "configure file in protobuf format load error: path = " << FLAGS_routing_map_config_path;
        return false;
    }
    apollo::routing::GraphCreator graph_creator(base_map_file_path, routing_map_file_path, conf);
    if (!graph_creator.Create()) {
        AERROR << "Generate routing map failed";
        return false;
    }
    return true;
}

bool CovertJunctions(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &junction : editor_map.junction()) {
        auto *junction_pb = map->add_junction();
        junction_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::Junction>(junction.id()));
        junction_pb->mutable_polygon()->CopyFrom(GetJunctionPolygon(editor_map, junction.boundary_id()));
        apollo::hdmap::Junction::Type junction_type;
        if (!GetJunctionType(junction, junction_type)) {
            return false;
        }
        junction_pb->set_type(junction_type);
    }
    return true;
}

bool CovertSpeedBumps(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &speed_bump : editor_map.speed_bump()) {
        auto *speed_bump_pb = map->add_speed_bump();
        auto *position = speed_bump_pb->add_position();
        speed_bump_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::SpeedBump>(speed_bump.id()));
        GetSpeedBumpCurve(editor_map, speed_bump, speed_bump.boundary_id(), position);
    }
    return true;
}

bool CovertCrosswalks(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &crosswalk : editor_map.crosswalk()) {
        auto *crosswalk_pb = map->add_crosswalk();
        crosswalk_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::Crosswalk>(crosswalk.id()));
        crosswalk_pb->mutable_polygon()->CopyFrom(GetCrosswalkPolygon(editor_map, crosswalk.boundary_id()));
    }
    return true;
}

bool CovertParkingSpaces(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &parking_space : editor_map.parking_space()) {
        auto *parking_space_pb = map->add_parking_space();
        parking_space_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::ParkingSpace>(parking_space.id()));
        parking_space_pb->mutable_polygon()->CopyFrom(GetParkingPolygon(editor_map, parking_space.boundary_id()));
        parking_space_pb->set_heading(parking_space.heading());
    }
    return true;
}

bool CovertTrafficSignals(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &signal : editor_map.traffic_signal()) {
        auto *signal_pb = map->add_signal();
        signal_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::Signal>(signal.id()));
        signal_pb->set_type(GetSignalType(signal));
        for (const auto &sub_signal : signal.sub_signal()) {
            auto *sub_signal_pb = signal_pb->add_subsignal();
            sub_signal_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::Subsignal>(sub_signal.id()));
            sub_signal_pb->set_type(GetSubSignalType(sub_signal));
        }
        apollo::map_tool::StopLine stop_line;
        apollo::hdmap::Curve curve;
        if (!GetStopLineById(editor_map, signal.stop_line_id(), stop_line)) {
            return false;
        }
        GetStopLineCurve(editor_map, stop_line, &curve);
        auto *stop_line_pb = signal_pb->add_stop_line();
        stop_line_pb->CopyFrom(curve);
        signal_pb->mutable_boundary()->CopyFrom(GetSignalPolygon(editor_map, signal));
    }
    return true;
}

void ConvertCurveTolane(
        apollo::map_tool::EditorMap *editor_map,
        std::map<std::string, apollo::map_tool::LaneParser *> *lane_relation_parser) {
    apollo::map_tool::EditorMap editor_map_temp;
    editor_map_temp.CopyFrom(*editor_map);
    for (auto &lane : editor_map_temp.lane()) {
        if (lane.type() == apollo::map_tool::Lane_Type::Lane_Type_CURVE) {
            apollo::map_tool::Boundary left_boundary;
            apollo::map_tool::Point left_start_point;
            apollo::map_tool::Point left_end_point;
            std::vector<apollo::map_tool::Position> left_interpolation_points;
            // Calculate left curve interpolation points.
            GetBoundaryByBoundaryId(editor_map_temp, lane.left_boundary_id(), left_boundary);
            GetPointByPointId(editor_map_temp, left_boundary.point_id(0), left_start_point);
            GetPointByPointId(editor_map_temp, left_boundary.point_id(1), left_end_point);

            apollo::map_tool::Boundary right_boundary;
            apollo::map_tool::Point right_start_point;
            apollo::map_tool::Point right_end_point;
            std::vector<apollo::map_tool::Position> right_interpolation_points;
            // Calculate right curve interpolation points.
            GetBoundaryByBoundaryId(editor_map_temp, lane.right_boundary_id(), right_boundary);
            GetPointByPointId(editor_map_temp, right_boundary.point_id(0), right_start_point);
            GetPointByPointId(editor_map_temp, right_boundary.point_id(1), right_end_point);

            apollo::common::math::Vec2d left_start(left_start_point.position().x(), left_start_point.position().y());
            apollo::common::math::Vec2d left_end(left_end_point.position().x(), left_end_point.position().y());
            apollo::common::math::Vec2d right_start(right_start_point.position().x(), right_start_point.position().y());
            apollo::common::math::Vec2d right_end(right_end_point.position().x(), right_end_point.position().y());
            double step_left = left_start.DistanceTo(left_end) / FLAGS_curve_sampling_rate;
            double step_right = right_start.DistanceTo(right_end) / FLAGS_curve_sampling_rate;
            double step = step_left > step_right ? step_left : step_right;
            GetInterpolationCurvePoints(
                    right_start_point.position(),
                    right_end_point.position(),
                    right_boundary.controls_position(0),
                    right_boundary.controls_position(1),
                    &right_interpolation_points,
                    step);
            GetInterpolationCurvePoints(
                    left_start_point.position(),
                    left_end_point.position(),
                    left_boundary.controls_position(0),
                    left_boundary.controls_position(1),
                    &left_interpolation_points,
                    step);
            AddInterpolationToCurveBoundary(lane.left_boundary_id(), left_interpolation_points, editor_map);
            AddInterpolationToCurveBoundary(lane.right_boundary_id(), right_interpolation_points, editor_map);
        }
    }
}

bool CovertHeader(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    if (!editor_map.has_header()) {
        return false;
    }
    auto *header_pb = map->mutable_header();
    header_pb->set_version(editor_map.header().version());
    header_pb->set_date(editor_map.header().date());
    return true;
}

bool CovertStopSigns(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &stop_sign : editor_map.stop_sign()) {
        auto *stop_sign_pb = map->add_stop_sign();
        stop_sign_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::StopSign>(stop_sign.id()));
        apollo::hdmap::Curve curve;
        apollo::map_tool::StopLine stop_line;
        if (!GetStopLineById(editor_map, stop_sign.stop_line_id(), stop_line)) {
            return false;
        }
        GetStopLineCurve(editor_map, stop_line, &curve);
        stop_sign_pb->add_stop_line()->CopyFrom(curve);
    }
    return true;
}

bool CovertYieldSigns(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    for (const auto &yield_sign : editor_map.yield_sign()) {
        auto *yield_sign_pb = map->add_yield();
        yield_sign_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::YieldSign>(yield_sign.id()));
        apollo::hdmap::Curve curve;
        apollo::map_tool::StopLine stop_line;
        if (!GetStopLineById(editor_map, yield_sign.stop_line_id(), stop_line)) {
            return false;
        }
        GetStopLineCurve(editor_map, stop_line, &curve);
        yield_sign_pb->add_stop_line()->CopyFrom(curve);
    }
    return true;
}

bool CovertArea(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    // In Apollo 8.0, hdmap has ClearArea instead of Area.
    for (const auto &area : editor_map.area()) {
        auto *clear_area_pb = map->add_clear_area();
        clear_area_pb->mutable_id()->CopyFrom(GetMapElementId<apollo::hdmap::ClearArea>(area.id()));
        clear_area_pb->mutable_polygon()->CopyFrom(GetPolygonByBoundaryId(editor_map, area.boundary_id()));
        // Apollo 8.0 ClearArea does not have type/name fields.
    }
    return true;
}

bool CovertBarrierGate(const apollo::map_tool::EditorMap &editor_map, apollo::hdmap::Map *map) {
    // Apollo 8.0 hdmap does not define BarrierGate. No conversion performed.
    (void)editor_map;
    (void)map;
    return true;
}

bool VerifyMapValidity(apollo::hdmap::Map &map, ValidatorResult *result) {
    std::shared_ptr<MapValidator> map_validator;
    map_validator = std::make_shared<MapValidator>();
    const std::string checker_config_path = ResolveMapToolConfPath(FLAGS_default_checker_config_path);
    const std::string errcode_config_path = ResolveMapToolConfPath(FLAGS_default_errcode_config_path);
    bool ret = map_validator->Init(checker_config_path, errcode_config_path, map);
    if (!ret) {
        AERROR << "Map verify map validity failed.";
        return false;
    }
    map_validator->Check(*result);
    return true;
}

bool OpenGeoInfoFile(const std::string &base_map_dir, Json *geo_file) {
    std::ifstream file(base_map_dir + "/map_images/" + GEO_INFO_FILE);
    if (!file.is_open()) {
        AERROR << "The map doesn't have " << GEO_INFO_FILE;
        return false;
    }
    *geo_file = Json::parse(file);
    file.close();
    return true;
}

}  // namespace map_tool
}  // namespace apollo
