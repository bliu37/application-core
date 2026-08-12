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

#include "modules/map_creator/tile_map_images_creator/time_align/vector_align_adapter.hpp"

#include "gtest/gtest.h"

#include "boost/format.hpp"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/map_creator/tile_map_images_creator/time_align/record_align_adapter.h"

namespace apollo {
namespace tile_map_images_creator {

class VectorAlignAdapterTest : public testing::Test {
public:
    virtual void SetUp() override {
        records = {"/media/apollo_record/1016/small/20231016160540.record.00008"};
    }

    std::vector<std::string> records;
};

TEST_F(VectorAlignAdapterTest, create_vector) {
    std::shared_ptr<RecordAlignAdapter<apollo::localization::LocalizationEstimate>> p_record_adapter
            = std::make_shared<RecordAlignAdapter<apollo::localization::LocalizationEstimate>>(records);
    p_record_adapter->SetChannels("/apollo/localization/pose");
    p_record_adapter->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::localization::LocalizationEstimate>& x) -> u_int64_t {
                unsigned long long ll_i = (unsigned long long)x->measurement_time();
                unsigned long long ll_f = (x->measurement_time() - ll_i) * 1e9;
                return ll_i * 1000000000LL + ll_f;
            });

    std::tuple<std::shared_ptr<apollo::localization::LocalizationEstimate>> frame;
    uint64_t timestamp;
    std::vector<std::shared_ptr<apollo::localization::LocalizationEstimate>> localization_vec;

    while (p_record_adapter->GetNextFrame(frame, timestamp)) {
        std::shared_ptr<apollo::localization::LocalizationEstimate> localization_frame = std::get<0>(frame);
        localization_vec.emplace_back(localization_frame);
    }

    std::shared_ptr<VectorAlignAdapter<apollo::localization::LocalizationEstimate>> p_vector_adapter
            = std::make_shared<VectorAlignAdapter<apollo::localization::LocalizationEstimate>>();
    AINFO << "localization frame load, nums = " << localization_vec.size();

    p_vector_adapter->Init(localization_vec);
    p_vector_adapter->SetTimestampFunctor(
            [](const std::shared_ptr<apollo::localization::LocalizationEstimate>& x) -> u_int64_t {
                unsigned long long ll_i = (unsigned long long)x->measurement_time();
                unsigned long long ll_f = (x->measurement_time() - ll_i) * 1e9;
                return ll_i * 1000000000LL + ll_f;
            });

    std::tuple<std::shared_ptr<apollo::localization::LocalizationEstimate>> new_frame;

    bool ret = p_vector_adapter->GetNextFrameWithTimestamp(new_frame, 1697443923170000075ULL);
    EXPECT_TRUE(ret);
    std::shared_ptr<apollo::localization::LocalizationEstimate> actual_frame = std::get<0>(new_frame);
    AINFO << boost::format("get frame for timestamp 1697443923170000075, timestamp = %.9lf")
                    % actual_frame->measurement_time();
}

}  // namespace tile_map_images_creator
}  // namespace apollo