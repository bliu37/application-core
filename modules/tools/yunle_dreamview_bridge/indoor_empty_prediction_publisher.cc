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

#include <memory>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kPredictionTopic[] = "/apollo/prediction";

}  // namespace

// Indoor navigation does not run perception/prediction yet. This component
// publishes an empty PredictionObstacles message so Apollo Planning receives
// the same no-obstacle input used by the validated preflight.
class YunleIndoorEmptyPredictionPublisher final
    : public apollo::cyber::TimerComponent {
 public:
  bool Init() override {
    prediction_writer_ =
        node_->CreateWriter<prediction::PredictionObstacles>(kPredictionTopic);
    if (prediction_writer_ == nullptr) {
      AERROR << "Failed to create indoor empty prediction writer on "
             << kPredictionTopic;
      return false;
    }

    AINFO << "Yunle indoor empty Prediction publisher started on "
          << kPredictionTopic << ".";
    return true;
  }

  bool Proc() override {
    auto prediction_msg = std::make_shared<prediction::PredictionObstacles>();
    common::util::FillHeader(node_->Name(), prediction_msg.get());
    prediction_writer_->Write(prediction_msg);
    return true;
  }

 private:
  std::shared_ptr<cyber::Writer<prediction::PredictionObstacles>>
      prediction_writer_;
};

CYBER_REGISTER_COMPONENT(YunleIndoorEmptyPredictionPublisher)

}  // namespace yunle
}  // namespace apollo
