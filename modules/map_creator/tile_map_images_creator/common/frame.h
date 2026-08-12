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

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

struct PointXYZIT_D {
    double x;
    double y;
    double z;
    int intensity;
    unsigned long long timestamp;
};

/**
 * @brief the basic unit for point cloud data transforming and rendering
 */
struct ImageCreatorFrame {
    std::shared_ptr<apollo::drivers::PointCloud> pcd_frame_;
    std::shared_ptr<apollo::localization::LocalizationEstimate> localization_frame_;
};

struct ResultFrame {
    std::shared_ptr<std::vector<PointXYZIT_D>> pcd_frame_;
    std::shared_ptr<apollo::localization::LocalizationEstimate> localization_frame_;
};