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
#include "modules/map_creator/tile_map_images_creator/matrix_generator/matrix_generator.h"

#include <boost/format.hpp>

#include "cyber/cyber.h"

namespace apollo {
namespace tile_map_images_creator {

void MatrixGenerator::Init(ImagesCreatorConf conf) {
    maps_.clear();
    conf_ = conf;
}

void MatrixGenerator::AddMapOption(double resolution, int resolution_id) {
    std::shared_ptr<GridOption> opt_ptr = std::make_shared<GridOption>();

    // 参考自multimap_create_mapper.cpp文件
    opt_ptr->set_min_x(0.0);
    opt_ptr->set_min_y(0.0);
    opt_ptr->set_min_z(-10000.0);
    opt_ptr->set_matrix_size(1024);
    opt_ptr->set_resolution(resolution);
    opt_ptr->set_resolution_id(resolution_id);
    opt_ptr->set_zone_id(0);

    AddMapOption(opt_ptr);
    AINFO << "Add Map, resolution = " << resolution << ", id = " << resolution_id;
}

void MatrixGenerator::AddMapOption(std::shared_ptr<GridOption> option) {
    std::shared_ptr<GridMap> map_ptr = std::make_shared<GridMap>(option, conf_);

    std::string image_output_path = conf_.input_output_conf().images_output_dir();
    std::string images_path = boost::str(boost::format("%s/%d") % image_output_path % option->resolution_id);
    cyber::common::EnsureDirectory(images_path);

    maps_.emplace_back(map_ptr);
}

bool MatrixGenerator::InsertPointCloud(const std::vector<PointXYZIT_D> &points, const common::PointENU& now_localization) {
    int point_num = points.size();
    for (const PointXYZIT_D &point : points) {
        double x = point.x;
        double y = point.y;
        double z = point.z;
        double intensity = point.intensity;

        for (std::shared_ptr<GridMap> &single_map : maps_) {
            if (!single_map->SetValue(x, y, z, z - now_localization.z(), z - now_localization.z(), z - now_localization.z(), 1, intensity, intensity, intensity, intensity)) {
                AERROR << boost::format(
                                  "point cloud insert error on map_id = %d with (x, y, z, d) = (%.10f, %.10f, %.4f, "
                                  "%d)")
                                % single_map->get_resolution_id() % x % y % z % (int)intensity;
                return false;
            }
            ADEBUG << boost::format("insert point on map_id = %d with (x, y, z, d) = (%.10f, %.10f, %.4f, %d)")
                            % single_map->get_resolution_id() % x % y % z % (int)intensity;
        }
    }
    AINFO << point_num << " points in pcl have been inserted";
    return true;
}

bool MatrixGenerator::SaveToDisk() {
    AINFO << __FUNCTION__ << " start, _maps.size(): " << maps_.size();
    bool all_success = true;
    for (std::shared_ptr<GridMap> map_ptr : maps_) {
        for (auto iter : map_ptr->get_matrixs()) {
            if (!map_ptr->SaveMatrixToDisk(iter.first)) {
                MatrixIndex index = iter.first;
                AERROR << boost::format("save value to disk fail, id = %d , index = (%d, %d)")
                                % map_ptr->get_resolution_id() % index.index_x_ % index.index_y_;
                all_success = false;
            }
        }
    }
    return all_success;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
