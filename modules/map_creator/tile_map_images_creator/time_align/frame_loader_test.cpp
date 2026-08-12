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

#include "modules/map_creator/tile_map_images_creator/time_align/frame_loader.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"
#include "modules/map_creator/tile_map_images_creator/proto/slam_pose_result.pb.h"

#include "modules/map_creator/tile_map_images_creator/common/utils.h"
#include "modules/map_creator/tile_map_images_creator/time_align/multi_source_frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/record_align_adapter.h"
#include "modules/map_creator/tile_map_images_creator/time_align/single_source_frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/vector_align_adapter.hpp"

#define TF_ADAPTER RecordAlignAdapter<apollo::transform::TransformStampeds>

namespace apollo {
namespace tile_map_images_creator {

class FrameLoaderTest : public testing::Test {
public:
    virtual void SetUp() override {
        records = {"/media/apollo_record/1016/small/20231016160540.record.00008"};
        localization_bin_path = "/apollo_workspace/data/localization_pose.bin";
    }

    std::vector<std::string> records;
    std::string localization_bin_path;
};

TEST_F(FrameLoaderTest, create_localization_bin) {
    std::shared_ptr<TF_ADAPTER> p_adapter = std::make_shared<TF_ADAPTER>(records);
    EXPECT_NE(p_adapter, nullptr);

    p_adapter->SetChannels("/tf");

    p_adapter->SetTimestampFunctor([](const std::shared_ptr<apollo::transform::TransformStampeds>& x) -> uint64_t {
        return GetNanosecondTimestampFromSecondTimestamp(x->header().timestamp_sec());
    });

    apollo::tile_map_images_creator::SlamPoseResult slam_result;

    std::tuple<std::shared_ptr<apollo::transform::TransformStampeds>> frame;
    uint64_t timestamp;

    while (p_adapter->GetNextFrame(frame, timestamp)) {
        std::shared_ptr<apollo::transform::TransformStampeds> localization_frame = std::get<0>(frame);
        auto tf_result = slam_result.add_slam_pose_result();
        tf_result->CopyFrom(localization_frame->transforms(0));
    }

    EXPECT_GT(slam_result.slam_pose_result_size(), 2000);

    bool ret = apollo::cyber::common::SetProtoToBinaryFile(slam_result, localization_bin_path);
    EXPECT_TRUE(ret);
}

TEST_F(FrameLoaderTest, multi_loader_test) {
    std::shared_ptr<FrameLoaderInterface> frame_loader = std::make_shared<MultiSourceFrameLoader>(
            records, localization_bin_path, "/apollo/sensor/rslidar32/compensator/PointCloud2");
    EXPECT_NE(frame_loader, nullptr);
    bool ret = frame_loader->Init();
    EXPECT_TRUE(ret);

    ImageCreatorFrame frame;

    double pcd_time = 0.0, localization_time = 0.0;

    int cnt = 0;
    while (frame_loader->LoadNextFrame(frame)) {
        ++cnt;
        AINFO << boost::format("frame load from multi source, pcd timestamp = %.9lf, localization timestamp = %.9lf")
                        % frame.pcd_frame_->measurement_time() % frame.localization_frame_->measurement_time();
        EXPECT_GT(frame.pcd_frame_->measurement_time(), pcd_time);
        EXPECT_GT(frame.localization_frame_->measurement_time(), localization_time);

        pcd_time = frame.pcd_frame_->measurement_time();
        localization_time = frame.localization_frame_->measurement_time();
    }

    EXPECT_GT(cnt, 400);
    AINFO << "load " << cnt << " frames";
}

TEST_F(FrameLoaderTest, single_loader_test) {
    std::shared_ptr<FrameLoaderInterface> frame_loader = std::make_shared<SingleSourceFrameLoader>(
            records, "/apollo/localization/pose", "/apollo/sensor/rslidar32/compensator/PointCloud2");
    EXPECT_NE(frame_loader, nullptr);
    bool ret = frame_loader->Init();
    EXPECT_TRUE(ret);

    ImageCreatorFrame frame;

    double pcd_time = 0.0, localization_time = 0.0;
    int cnt = 0;
    while (frame_loader->LoadNextFrame(frame)) {
        ++cnt;
        AINFO << boost::format(
                         "frame load from single record source, pcd timestamp = %.9lf, localization timestamp = %.9lf")
                        % frame.pcd_frame_->measurement_time() % frame.localization_frame_->measurement_time();
        EXPECT_GT(frame.pcd_frame_->measurement_time(), pcd_time);
        EXPECT_GT(frame.localization_frame_->measurement_time(), localization_time);

        pcd_time = frame.pcd_frame_->measurement_time();
        localization_time = frame.localization_frame_->measurement_time();
    }

    EXPECT_GT(cnt, 400);
    AINFO << "load " << cnt << " frames";
}

}  // namespace tile_map_images_creator
}  // namespace apollo
