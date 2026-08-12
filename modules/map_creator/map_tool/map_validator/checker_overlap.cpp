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

#include "checker_overlap.h"

#include <boost/format.hpp>

namespace apollo {
namespace map_tool {

bool CheckerOverlap::Check(std::vector<ErrorMessage>& err_messages) {
  lane_id_set_.clear();
  for (int i = 0; i < hdmap_message_->lane_size(); ++i) {
    const std::string& lane_id = hdmap_message_->lane(i).id().id();
    lane_id_set_.emplace(lane_id);
  }

  overlap_lane_exist_mask_.clear();
  for (const hdmap::Overlap& overlap : hdmap_message_->overlap()) {
    const std::string& overlap_id = overlap.id().id();
    for (const hdmap::ObjectOverlapInfo& overlap_object : overlap.object()) {
      if (overlap_object.has_lane_overlap_info()) {
        const auto& overlap_lane_id = overlap_object.id().id();
        // const auto& lane_overlap_info = overlap_object.lane_overlap_info();
        if (lane_id_set_.count(overlap_lane_id)) {
          overlap_lane_exist_mask_.insert(overlap_id);
          ADEBUG << boost::format("overlap id = %s associate to lane id = %d") %
                        overlap_id % overlap_lane_id;
        } else {
          AERROR
              << boost::format(
                     "overlap id = %s has an non-exist overlap lane id = %d") %
                     overlap_id % overlap_lane_id;
        }
      }
    }
  }

  // 红绿灯overlap
  if (conf_.enable_signal_overlap_lane_check()) {
    for (const hdmap::Signal& signal : hdmap_message_->signal()) {
      bool exist_associate_lane_id = false;
      if (std::none_of(signal.overlap_id().begin(), signal.overlap_id().end(),
                       [this](const hdmap::Id& id) {
                         return overlap_lane_exist_mask_.count(id.id()) > 0;
                       })) {
        AERROR << "signal id = " << signal.id().id()
               << " cannot associate any lane id";
        RETURN_VAL_IF(
            !AddNewErrorMessage(err_messages, SignalCannotAssociateLaneError,
                                signal.id().id()),
            false);
      }
    }
  }

  // 减速带
  if (conf_.enable_speed_bump_overlap_lane_check()) {
    for (const hdmap::SpeedBump& speed_bump : hdmap_message_->speed_bump()) {
      if (std::none_of(speed_bump.overlap_id().begin(),
                       speed_bump.overlap_id().end(),
                       [this](const hdmap::Id& id) {
                         return overlap_lane_exist_mask_.count(id.id()) > 0;
                       })) {
        AERROR << "speed bump id = " << speed_bump.id().id()
               << " cannot associate any lane id";
        RETURN_VAL_IF(
            !AddNewErrorMessage(err_messages, SpeedBumpCannotAssociateLaneError,
                                speed_bump.id().id()),
            false);
      }
    }
  }

  // 停车位
  if (conf_.enable_parking_overlap_lane_check()) {
    for (const hdmap::ParkingSpace& parking_space :
         hdmap_message_->parking_space()) {
      if (std::none_of(parking_space.overlap_id().begin(),
                       parking_space.overlap_id().end(),
                       [this](const hdmap::Id& id) {
                         return overlap_lane_exist_mask_.count(id.id()) > 0;
                       })) {
        AERROR << "parking space id = " << parking_space.id().id()
               << " cannot associate any lane id";
        RETURN_VAL_IF(!AddNewErrorMessage(err_messages,
                                          ParkingSpaceCannotAssociateLaneError,
                                          parking_space.id().id()),
                      false);
      }
    }
  }

  // 人行横道
  if (conf_.enable_cross_walk_overlap_lane_check()) {
    for (const hdmap::Crosswalk& crosswalk : hdmap_message_->crosswalk()) {
      if (std::none_of(crosswalk.overlap_id().begin(),
                       crosswalk.overlap_id().end(),
                       [this](const hdmap::Id& id) {
                         return overlap_lane_exist_mask_.count(id.id()) > 0;
                       })) {
        AERROR << "crosswalk id = " << crosswalk.id().id()
               << " cannot associate any lane id";
        RETURN_VAL_IF(
            !AddNewErrorMessage(err_messages, CrossWalkCannotAssociateLaneError,
                                crosswalk.id().id()),
            false);
      }
    }
  }

  // 让行标志牌
  if (conf_.enable_yield_sign_overlap_lane_check()) {
    for (const hdmap::YieldSign& yield : hdmap_message_->yield()) {
      if (std::none_of(yield.overlap_id().begin(), yield.overlap_id().end(),
                       [this](const hdmap::Id& id) {
                         return overlap_lane_exist_mask_.count(id.id()) > 0;
                       })) {
        AERROR << "yield sign id = " << yield.id().id()
               << " cannot associate any lane id";
        RETURN_VAL_IF(
            !AddNewErrorMessage(err_messages, YieldSignCannotAssociateLaneError,
                                yield.id().id()),
            false);
      }
    }
  }

  // 禁行标志牌
  if (conf_.enable_stop_sign_overlap_lane_check()) {
    for (const hdmap::StopSign& stopsign : hdmap_message_->stop_sign()) {
      if (std::none_of(stopsign.overlap_id().begin(),
                       stopsign.overlap_id().end(),
                       [this](const hdmap::Id& id) {
                         return overlap_lane_exist_mask_.count(id.id()) > 0;
                       })) {
        AERROR << "stop sign id = " << stopsign.id().id()
               << " cannot associate any lane id";
        RETURN_VAL_IF(
            !AddNewErrorMessage(err_messages, StopSignCannotAssociateLaneError,
                                stopsign.id().id()),
            false);
      }
    }
  }

  // 道闸
  // if (conf_.enable_barrier_gate_overlap_lane_check()) {
  //     for (const hdmap::BarrierGate& barrier_gate :
  //     hdmap_message_->barrier_gate()) {
  //         if (std::none_of(
  //                     barrier_gate.overlap_id().begin(),
  //                     barrier_gate.overlap_id().end(),
  //                     [this](const hdmap::Id& id) { return
  //                     overlap_lane_exist_mask_.count(id.id()) > 0; })) {
  //             AERROR << "barrier gate id = " << barrier_gate.id().id() << "
  //             cannot associate any lane id"; RETURN_VAL_IF(
  //                     !AddNewErrorMessage(err_messages,
  //                     BarrierGateCannotAssociateLaneError,
  //                     barrier_gate.id().id()), false);
  //         }
  //     }
  // }

  return true;
}

}  // namespace map_tool
}  // namespace apollo
