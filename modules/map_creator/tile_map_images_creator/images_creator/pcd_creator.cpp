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

#include "modules/map_creator/tile_map_images_creator/images_creator/pcd_creator.h"
#include "cyber/cyber.h"

namespace apollo {
namespace tile_map_images_creator {

void PcdCreator::ConcatPointCloud(const std::vector<PointXYZIT_D> &points) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    for (unsigned int i = 0; i < points.size(); ++i) {
        double x = points[i].x;
        double y = points[i].y;
        double z = points[i].z;
        double intensity = points[i].intensity;

        if (i == 0 && all_cloud_->empty()) {
            base_x_ = x;
            base_y_ = y;
        }

        pcl::PointXYZI point = pcl::PointXYZI(x - base_x_, y - base_y_, z, intensity);
        pcl_cloud.push_back(point);
    }

    if (pcl_cloud.empty()) {
        AWARN << "No need to add empty cloud";
        return;
    }
    AINFO << "point cloud concat with " << pcl_cloud.size() << " points";
    *all_cloud_ += pcl_cloud;
}

void PcdCreator::SaveCloudToPcd(const std::string &path) {
    if (all_cloud_->empty()) {
        AERROR << "Fail to SaveCloudToPcd, allCloud_ empty";
        return;
    }
    // assert(pcl::io::savePCDFileBinary(path, all_cloud_) == 0);
    int ret_code = 0;
    if ((ret_code = pcl::io::savePCDFileBinary(path, *all_cloud_)) != 0) {
        AERROR << "Fail to save pcd file to path " << path << ", code = " << ret_code;
    }
    AINFO << "whole point cloud file save, path = " << path << ", point = " << all_cloud_->size();
}

}  // namespace tile_map_images_creator
}  // namespace apollo