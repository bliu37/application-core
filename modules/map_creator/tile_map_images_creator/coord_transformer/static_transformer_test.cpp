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

#include "static_transformer.h"
#include "gtest/gtest.h"
#include "cyber/cyber.h"
#include "boost/format.hpp"
#include <memory>

TEST(StaticTransformerTest, getting_static_tf_test) {
    apollo::cyber::Init("static transformer test");
    FLAGS_alsologtostderr = 1;

    apollo::tile_map_images_creator::StaticTransformer* p_static_tf
            = apollo::tile_map_images_creator::StaticTransformer::Instance();
    apollo::tile_map_images_creator::CombinedTransform tf;
    EXPECT_NE(p_static_tf, nullptr);
    p_static_tf->GetStaticTransform2Localization("rslidar32_up", tf);

    AINFO << boost::format(
                     "getting static tf, transalte(x, y, z) = (%.6f, %.6f, %.6f), rotation(qx, qy, qz, qw) = (%.6f, "
                     "%.6f, %.6f, %.6f)")
                    % tf.translation.x() % tf.translation.y() % tf.translation.z() % tf.rotation.x() % tf.rotation.y()
                    % tf.rotation.z() % tf.rotation.w();
}
