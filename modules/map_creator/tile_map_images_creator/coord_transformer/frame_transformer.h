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

#include "static_transformer.h"
#include "modules/map_creator/tile_map_images_creator/common/frame.h"
#include "Eigen/Eigen"

namespace apollo {
namespace tile_map_images_creator {

/**
 * @brief Convert point cloud to world coordinate according to static transform and the provided dynamic transform.
 */
class FrameTransformer {
public:
    FrameTransformer() = default;

    /**
     * @brief Convert point cloud in lidar coordinate to world coordinate.
     * @param frame. Contain source point cloud and localization pose frame. localization pose frame can provide dynamic
     * transform from localization coordinate to world coordinate.
     * @param result_pcd. point cloud result info in world coordinate
     * @return return true if transform success
     * @note PLEASE LAUNCH APOLLO TRANSFORM MODULE BEFORE CALLING THIS FUNCTION. Because static transform rely on
     * transform module.
     */
    bool Transform(const ImageCreatorFrame& source_frame, ResultFrame& result_frame);

    /**
     * @brief Get static transform in providing pose frame, which can convert point cloud from lidar coordinate to
     * localization coordinate.
     * @param pose_frame. localization pose frame
     * @return transform. Including translation and rotation of the transform.
     * @return return true if transform success.
     */
    bool GetTransformFromPose(
            const std::shared_ptr<apollo::localization::LocalizationEstimate>& pose_frame,
            CombinedTransform& transform);

private:
    StaticTransformer* static_transformer_instance_ = StaticTransformer::Instance();
};

}  // namespace tile_map_images_creator
}  // namespace apollo
