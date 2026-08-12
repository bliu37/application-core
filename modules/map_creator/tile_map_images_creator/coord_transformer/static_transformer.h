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
#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/transform/buffer.h"

namespace apollo {
namespace tile_map_images_creator {

class CombinedTransform {
public:
    Eigen::Translation3d translation;

    Eigen::Quaterniond rotation;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StaticTransformer {
public:
    /**
     * @brief Get static transform from the coordinate of given frame id to the one of localization
     * @param child_frame_id. given frame id.
     * @return tf. result transform
     * @return return true if transform success
     */
    bool GetStaticTransform2Localization(const std::string& child_frame_id, CombinedTransform& tf);

    void SetLocalizationFrameId(const std::string& localization_frame_id) {
        localization_frame_id_ = localization_frame_id;
    }

private:
    // cache static transform of each child frame id
    std::map<
            std::string,
            CombinedTransform,
            std::less<std::string>,
            Eigen::aligned_allocator<std::pair<const std::string, CombinedTransform>>>
            transform_map;

    std::string localization_frame_id_ = "localization";
    apollo::transform::Buffer* tf2_buffer_ = apollo::transform::Buffer::Instance();

    DECLARE_SINGLETON(StaticTransformer);
};

}  // namespace tile_map_images_creator
}  // namespace apollo