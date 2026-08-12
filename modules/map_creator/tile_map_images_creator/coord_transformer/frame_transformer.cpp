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
#include "modules/map_creator/tile_map_images_creator/coord_transformer/frame_transformer.h"

namespace apollo {
namespace tile_map_images_creator {

bool FrameTransformer::Transform(const ImageCreatorFrame& source_frame, ResultFrame& result_frame) {
    CombinedTransform static_tf;
    if (!static_transformer_instance_->GetStaticTransform2Localization(
                source_frame.pcd_frame_->header().frame_id(), static_tf)) {
        AERROR << "get static transform to localization fail";
        return false;
    }

    Eigen::Affine3d static_tf_matrix = static_tf.translation * static_tf.rotation;
    CombinedTransform dynamic_tf;
    if (!GetTransformFromPose(source_frame.localization_frame_, dynamic_tf)) {
        return false;
    }
    Eigen::Affine3d dynamic_tf_matrix = dynamic_tf.translation * dynamic_tf.rotation;
    Eigen::Affine3d tf_matrix = dynamic_tf_matrix * static_tf_matrix;

    std::shared_ptr<apollo::drivers::PointCloud> pcd_frame = source_frame.pcd_frame_;

    int point_size = pcd_frame->point_size();
    Eigen::MatrixXd points_origin;
    points_origin.resize(point_size, 3);
    for (int i = 0; i < point_size; ++i) {
        points_origin(i, 0) = pcd_frame->point(i).x();
        points_origin(i, 1) = pcd_frame->point(i).y();
        points_origin(i, 2) = pcd_frame->point(i).z();
    }

    Eigen::MatrixXd points_result;
    points_result.resize(point_size, 4);
    points_result = tf_matrix.matrix() * points_origin.rowwise().homogeneous().transpose();

    //    result_pcd = std::make_shared<apollo::drivers::PointCloud>();
    std::shared_ptr<std::vector<PointXYZIT_D>> result_pcd = std::make_shared<std::vector<PointXYZIT_D>>();
    for (int i = 0; i < point_size; ++i) {
        PointXYZIT_D result_point;
        result_point.x = points_result.matrix()(0, i);
        result_point.y = points_result.matrix()(1, i);
        result_point.z = points_result.matrix()(2, i);
        result_point.intensity = pcd_frame->point(i).intensity();
        result_pcd->emplace_back(result_point);
    }
    result_frame.pcd_frame_ = result_pcd;
    result_frame.localization_frame_ = source_frame.localization_frame_;

    return true;
}

bool FrameTransformer::GetTransformFromPose(
        const std::shared_ptr<apollo::localization::LocalizationEstimate>& pose_frame,
        CombinedTransform& transform) {
    if (!pose_frame->has_pose()) {
        AERROR << "pose frame lack of header";
        return false;
    }
    transform.translation.x() = pose_frame->pose().position().x();
    transform.translation.y() = pose_frame->pose().position().y();
    transform.translation.z() = pose_frame->pose().position().z();
    transform.rotation.x() = pose_frame->pose().orientation().qx();
    transform.rotation.y() = pose_frame->pose().orientation().qy();
    transform.rotation.z() = pose_frame->pose().orientation().qz();
    transform.rotation.w() = pose_frame->pose().orientation().qw();
    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
