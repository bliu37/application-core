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
#include "modules/common_msgs/localization_msgs/localization.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kLocalizationTopic[] = "/apollo/localization/pose";

}  // namespace

// This component is only a temporary aid for viewing raw sensor point clouds
// in Dreamview Plus before a real localization source is available. It must
// not run at the same time as the production localization module.
class YunleLocalizationTimeBridge final
    : public apollo::cyber::TimerComponent {
 public:
  bool Init() override {
    localization_writer_ =
        node_->CreateWriter<localization::LocalizationEstimate>(
            kLocalizationTopic);
    if (localization_writer_ == nullptr) {
      AERROR << "Failed to create preview localization writer on "
             << kLocalizationTopic;
      return false;
    }

    AWARN << "Dreamview preview bridge is publishing timestamp-only "
          << "localization messages. Stop this component before starting "
          << "real localization.";
    return true;
  }

  bool Proc() override {
    auto localization_msg =
        std::make_shared<localization::LocalizationEstimate>();
    common::util::FillHeader("yunle_dreamview_bridge",
                             localization_msg.get());
    localization_msg->set_measurement_time(
        localization_msg->header().timestamp_sec());

    // Keep the placeholder pose mathematically valid for Dreamview consumers.
    localization_msg->mutable_pose()->mutable_orientation()->set_qw(1.0);

    localization_writer_->Write(localization_msg);
    return true;
  }

 private:
  std::shared_ptr<
      apollo::cyber::Writer<localization::LocalizationEstimate>>
      localization_writer_;
};

CYBER_REGISTER_COMPONENT(YunleLocalizationTimeBridge)

}  // namespace yunle
}  // namespace apollo
