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
#include "modules/common_msgs/transform_msgs/transform.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr char kTfTopic[] = "/tf";
constexpr char kLocalizationFrame[] = "localization";
constexpr char kImuFrame[] = "imu";
constexpr char kLidarFrame[] = "lslidar16v4";

// Tape-measured JD03 mounting geometry. The CGI-230 axes are X left, Y rear,
// Z up; the LS-C16 V4 axes are X forward, Y left, Z up. The lidar origin is
// 0.03 m along IMU X, -0.33 m along IMU Y, and 0.67 m along IMU Z.
constexpr double kImuToLidarX = 0.03;
constexpr double kImuToLidarY = -0.33;
constexpr double kImuToLidarZ = 0.67;
constexpr double kSinMinus45Degrees = -0.7071067811865476;
constexpr double kCosMinus45Degrees = 0.7071067811865476;

void AddTransform(const char* parent_frame, const char* child_frame, double x,
                  double y, double z, double qx, double qy, double qz,
                  double qw, transform::TransformStampeds* transforms) {
  auto* stamped = transforms->add_transforms();
  stamped->mutable_header()->set_frame_id(parent_frame);
  stamped->set_child_frame_id(child_frame);
  stamped->mutable_transform()->mutable_translation()->set_x(x);
  stamped->mutable_transform()->mutable_translation()->set_y(y);
  stamped->mutable_transform()->mutable_translation()->set_z(z);
  stamped->mutable_transform()->mutable_rotation()->set_qx(qx);
  stamped->mutable_transform()->mutable_rotation()->set_qy(qy);
  stamped->mutable_transform()->mutable_rotation()->set_qz(qz);
  stamped->mutable_transform()->mutable_rotation()->set_qw(qw);
}

}  // namespace

// Temporary compatibility aid for stationary JD03 indoor-SLAM bring-up.
//
// Apollo's packaged StaticTransformComponent writes /tf_static only once. In
// the current Cyber environment LIORF's typed static-TF reader does not receive
// that sample reliably. Repeatedly writing /tf_static is also inappropriate
// because Apollo's Buffer retains every received static message without
// deduplication. This timer therefore publishes the fixed test transform on
// /tf with a current timestamp, allowing LIORF's Time(0) lookup to use the
// latest sample without growing its static-message cache.
//
// The localization frame is intentionally defined at the IMU origin with the
// same axes, so localization -> imu is an identity by definition. The
// imu -> lslidar16v4 transform uses tape-measured mounting geometry, not a
// calibrated extrinsic. Refine that transform before producing a formal map.
class YunleIndoorTestTfPublisher final
    : public apollo::cyber::TimerComponent {
 public:
  bool Init() override {
    writer_ = node_->CreateWriter<transform::TransformStampeds>(kTfTopic);
    if (writer_ == nullptr) {
      AERROR << "Failed to create the Yunle indoor test TF writer on "
             << kTfTopic;
      return false;
    }

    AddTransform(kLocalizationFrame, kImuFrame, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, &transform_stampeds_);
    AddTransform(kImuFrame, kLidarFrame, kImuToLidarX, kImuToLidarY,
                 kImuToLidarZ, 0.0, 0.0, kSinMinus45Degrees,
                 kCosMinus45Degrees, &transform_stampeds_);

    AWARN << "Publishing intentionally coincident " << kLocalizationFrame
          << " -> " << kImuFrame << " frames and tape-measured " << kImuFrame
          << " -> " << kLidarFrame << " transform on " << kTfTopic
          << ". Calibrate before formal map creation.";
    return true;
  }

  bool Proc() override {
    common::util::FillHeader(node_->Name(), &transform_stampeds_);
    for (auto& stamped : *transform_stampeds_.mutable_transforms()) {
      stamped.mutable_header()->set_timestamp_sec(
          transform_stampeds_.header().timestamp_sec());
      stamped.mutable_header()->set_sequence_num(
          transform_stampeds_.header().sequence_num());
    }
    writer_->Write(transform_stampeds_);
    return true;
  }

 private:
  std::shared_ptr<cyber::Writer<transform::TransformStampeds>> writer_;
  transform::TransformStampeds transform_stampeds_;
};

CYBER_REGISTER_COMPONENT(YunleIndoorTestTfPublisher)

}  // namespace yunle
}  // namespace apollo
