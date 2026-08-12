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

#include "modules/map_creator/tile_map_images_creator/time_align/record_align_adapter.h"

#include "gtest/gtest.h"

#include "boost/format.hpp"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#define ADAPTER RecordAlignAdapter<apollo::drivers::PointCloud, apollo::localization::LocalizationEstimate>

namespace apollo {
namespace tile_map_images_creator {

class RecordAlignAdapterTest : public testing::Test {
public:
    virtual void SetUp() override {
        records = {"/media/apollo_record/1016/small/20231016160540.record.00008"};
    }

    std::vector<std::string> records;
};

TEST_F(RecordAlignAdapterTest, get_frame_by_time_test) {
    std::shared_ptr<ADAPTER> p_adapter = std::make_shared<ADAPTER>(records);
    EXPECT_NE(p_adapter, nullptr);

    p_adapter->SetChannels("/apollo/sensor/rslidar32/compensator/PointCloud2", "/apollo/localization/pose");

    AINFO << "channel set";

    p_adapter->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::drivers::PointCloud>& x) -> uint64_t {
                unsigned long long ll_i = (unsigned long long)x->measurement_time();
                unsigned long long ll_f = (x->measurement_time() - ll_i) * 1e9;
                return ll_i * 1000000000LL + ll_f;
            },
            [](const std::shared_ptr<apollo::localization::LocalizationEstimate>& x) -> u_int64_t {
                unsigned long long ll_i = (unsigned long long)x->measurement_time();
                unsigned long long ll_f = (x->measurement_time() - ll_i) * 1e9;
                return ll_i * 1000000000LL + ll_f;
            });
    AINFO << "functor set";

    std::shared_ptr<apollo::drivers::PointCloud> pcd_frame = nullptr;
    std::shared_ptr<apollo::localization::LocalizationEstimate> localization_frame = nullptr;

    std::tuple<
            std::shared_ptr<apollo::drivers::PointCloud>,
            std::shared_ptr<apollo::localization::LocalizationEstimate>>
            frame;

    EXPECT_TRUE(p_adapter->GetNextFrameWithTimestamp(frame, 1697443888307576179ULL));

    std::tie(pcd_frame, localization_frame) = frame;

    EXPECT_NE(pcd_frame, nullptr);
    EXPECT_NE(localization_frame, nullptr);

    AINFO << boost::format("frame timestamp = %llu, pcl.timestamp %.9f, pose.timestamp %.9f") % 1697443888307576179ULL
                    % pcd_frame->measurement_time() % localization_frame->measurement_time();
}

TEST_F(RecordAlignAdapterTest, process_full_record_test) {
    std::shared_ptr<ADAPTER> p_adapter = std::make_shared<ADAPTER>(records);
    p_adapter->SetChannels("/apollo/sensor/rslidar32/compensator/PointCloud2", "/apollo/localization/pose");
    p_adapter->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::drivers::PointCloud>& x) -> uint64_t {
                unsigned long long ll_i = (unsigned long long)x->measurement_time();
                unsigned long long ll_f = (x->measurement_time() - ll_i) * 1e9;
                return ll_i * 1000000000LL + ll_f;
            },
            [](const std::shared_ptr<apollo::localization::LocalizationEstimate>& x) -> u_int64_t {
                unsigned long long ll_i = (unsigned long long)x->measurement_time();
                unsigned long long ll_f = (x->measurement_time() - ll_i) * 1e9;
                return ll_i * 1000000000LL + ll_f;
            });
    std::shared_ptr<apollo::drivers::PointCloud> pcd_frame = nullptr;
    std::shared_ptr<apollo::localization::LocalizationEstimate> localization_frame = nullptr;
    uint64_t timestamp = 0;

    std::tuple<
            std::shared_ptr<apollo::drivers::PointCloud>,
            std::shared_ptr<apollo::localization::LocalizationEstimate>>
            frame;

    double last_timestamp_pcd = 0.0, last_timestamp_localization = 0.0;
    uint64_t last_timestamp = 0;

    AINFO << "load all pcd frame start";

    while (p_adapter->GetNextFrame(frame, timestamp)) {
        std::tie(pcd_frame, localization_frame) = frame;
        EXPECT_NE(pcd_frame, nullptr);
        EXPECT_NE(localization_frame, nullptr);

        EXPECT_GT(timestamp, last_timestamp);
        EXPECT_GT(pcd_frame->measurement_time(), last_timestamp_pcd);
        EXPECT_GT(localization_frame->measurement_time(), last_timestamp_localization);

        last_timestamp = timestamp;
        last_timestamp_pcd = pcd_frame->measurement_time();
        last_timestamp_localization = localization_frame->measurement_time();

        AINFO << boost::format("frame timestamp = %llu, pcl.timestamp %.9f, pose.timestamp %.9f") % last_timestamp
                        % pcd_frame->measurement_time() % localization_frame->measurement_time();
    }

    AINFO << "test finished";
}

}  // namespace tile_map_images_creator
}  // namespace apollo
