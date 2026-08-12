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

#include "gtest/gtest.h"
#include "cyber/cyber.h"
#include "boost/format.hpp"
#include "modules/map_creator/tile_map_images_creator/time_align/frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/time_align/multi_source_frame_loader.h"
#include "modules/map_creator/tile_map_images_creator/coord_transformer/frame_transformer.h"

namespace apollo {
namespace tile_map_images_creator {

TEST(FrameTransformerTest, single_frame_tf_test) {
    cyber::Init("frame transformer test");
    FLAGS_alsologtostderr = 1;

    std::vector<std::string> records = {"/media/apollo_record/1016/small/20231016160540.record.00008"};
    std::string localization_bin_path = "/apollo_workspace/data/localization_pose.bin";
    std::shared_ptr<FrameTransformer> p_transformer = std::make_shared<FrameTransformer>();

    std::shared_ptr<FrameLoaderInterface> frame_loader = std::make_shared<MultiSourceFrameLoader>(
            records, localization_bin_path, "/apollo/sensor/rslidar32/compensator/PointCloud2");
    EXPECT_TRUE(frame_loader->Init());

    ImageCreatorFrame frame;
    frame_loader->LoadNextFrame(frame);

    ResultFrame result_frame;
    bool ret = p_transformer->Transform(frame, result_frame);
    EXPECT_TRUE(ret);

    AINFO << boost::format("pcd frame has been transformed, with %d points") % result_frame.pcd_frame_->size();

    while (frame_loader->LoadNextFrame(frame)) {
        bool ret = p_transformer->Transform(frame, result_frame);
        EXPECT_TRUE(ret);
        AINFO << boost::format("pcd frame has been transformed, with %d points") % result_frame.pcd_frame_->size();
        EXPECT_GT(result_frame.pcd_frame_->size(), 0);
    }
}

}  // namespace tile_map_images_creator
}  // namespace apollo