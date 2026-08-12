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
 *****************************************************************************/ \
#include "modules/map_creator/tile_map_images_creator/coord_transformer/static_transformer.h"

namespace apollo {
namespace tile_map_images_creator {

StaticTransformer::StaticTransformer() = default;

bool StaticTransformer::GetStaticTransform2Localization(const std::string& child_frame_id, CombinedTransform& tf) {
    if (transform_map.count(child_frame_id)) {
        tf = transform_map[child_frame_id];
        return true;
    }

    int overtime_counter = 0;

    const cyber::Time latest_time(0);
    std::string tf2_message;
    while (!tf2_buffer_->canTransform(
            localization_frame_id_, child_frame_id, latest_time, 0.02f, &tf2_message)) {
        sleep(1);
        overtime_counter++;
        AWARN << "Unable to query translation relationship from " << child_frame_id << " to " << localization_frame_id_
              << ". Try starting transform module, retry time = " << overtime_counter;
        if (cyber::IsShutdown() || overtime_counter > 3) {
            AERROR << "Unable to query static transform relationship. Try restarting transform module.";
            return false;
        }
    }

    apollo::transform::TransformStamped transform_relationship
            = tf2_buffer_->lookupTransform(localization_frame_id_, child_frame_id, latest_time, 0.02f);

    CombinedTransform combinedTransform;
    combinedTransform.translation.x() = transform_relationship.transform().translation().x();
    combinedTransform.translation.y() = transform_relationship.transform().translation().y();
    combinedTransform.translation.z() = transform_relationship.transform().translation().z();

    combinedTransform.rotation.x() = transform_relationship.transform().rotation().qx();
    combinedTransform.rotation.y() = transform_relationship.transform().rotation().qy();
    combinedTransform.rotation.z() = transform_relationship.transform().rotation().qz();
    combinedTransform.rotation.w() = transform_relationship.transform().rotation().qw();

    // Eigen::Affine3d transform = combinedTransform.translation * combinedTransform.rotation;

    tf = combinedTransform;
    transform_map[child_frame_id] = combinedTransform;

    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
