/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <memory>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kRawPlanningCommandTopic[] =
    "/apollo/yunle/indoor/raw_planning_command";
constexpr char kPlanningCommandTopic[] = "/apollo/planning/command";
constexpr double kStartWaypointBackoffM = 1.0;

}  // namespace

// Apollo LaneFollowMap treats the first routing request waypoint as a required
// waypoint to pass. Dreamview's indoor external_command path uses the current
// vehicle position as that first waypoint; on a short route, localization and
// projection jitter can leave the ADC just before it, preventing the real DEST
// stop wall from being generated. This adapter moves only the first waypoint
// and first segment start backward, keeping the selected destination unchanged.
class YunleIndoorPlanningCommandAdapter final
    : public cyber::Component<planning::PlanningCommand> {
 public:
  bool Init() override {
    planning_command_writer_ =
        node_->CreateWriter<planning::PlanningCommand>(kPlanningCommandTopic);
    if (planning_command_writer_ == nullptr) {
      AERROR << "Failed to create indoor PlanningCommand writer on "
             << kPlanningCommandTopic;
      return false;
    }
    AINFO << "Yunle indoor PlanningCommand adapter started: "
          << kRawPlanningCommandTopic << " -> " << kPlanningCommandTopic
          << ", start waypoint backoff=" << kStartWaypointBackoffM << " m.";
    return true;
  }

  bool Proc(const std::shared_ptr<planning::PlanningCommand>& input) override {
    if (input == nullptr) {
      return true;
    }
    auto output = std::make_shared<planning::PlanningCommand>();
    output->CopyFrom(*input);
    ApplyLaneFollowStartBackoff(output.get());
    planning_command_writer_->Write(output);
    return true;
  }

 private:
  void ApplyLaneFollowStartBackoff(planning::PlanningCommand* command) const {
    if (command == nullptr || !command->has_lane_follow_command()) {
      return;
    }
    auto* lane_follow = command->mutable_lane_follow_command();
    if (!lane_follow->has_routing_request() ||
        lane_follow->routing_request().waypoint_size() < 2) {
      return;
    }

    auto* request = lane_follow->mutable_routing_request();
    auto* first_waypoint = request->mutable_waypoint(0);
    if (!first_waypoint->has_s()) {
      return;
    }
    const double original_start_s = first_waypoint->s();
    const double adjusted_start_s =
        std::max(0.0, original_start_s - kStartWaypointBackoffM);
    first_waypoint->set_s(adjusted_start_s);

    if (lane_follow->road_size() > 0 &&
        lane_follow->mutable_road(0)->passage_size() > 0 &&
        lane_follow->mutable_road(0)->mutable_passage(0)->segment_size() > 0) {
      auto* first_segment =
          lane_follow->mutable_road(0)->mutable_passage(0)->mutable_segment(0);
      first_segment->set_start_s(
          std::min(first_segment->start_s(), adjusted_start_s));
    }

    AINFO << "Yunle adjusted indoor PlanningCommand start waypoint s from "
          << original_start_s << " to " << adjusted_start_s
          << "; destination is unchanged.";
  }

  std::shared_ptr<cyber::Writer<planning::PlanningCommand>>
      planning_command_writer_;
};

CYBER_REGISTER_COMPONENT(YunleIndoorPlanningCommandAdapter)

}  // namespace yunle
}  // namespace apollo
