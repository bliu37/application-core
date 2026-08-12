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
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/map_creator/tile_map_images_creator/common/frame.h"

namespace apollo {
namespace tile_map_images_creator {
class PcdCreator {
public:
    PcdCreator() {
        all_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }
    void ConcatPointCloud(const std::vector<PointXYZIT_D>& points);
    void SaveCloudToPcd(const std::string& path);

private:
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> all_cloud_;
    double base_x_, base_y_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo