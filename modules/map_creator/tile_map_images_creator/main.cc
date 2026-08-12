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

#include <chrono>
#include <future>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "absl/strings/str_cat.h"
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"

#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"
#include "modules/map_creator/tile_map_images_creator/images_creator/images_creator.h"
#include "modules/map_creator/tile_map_images_creator/config_loader/config_loader.h"

#include "cyber/cyber.h"

using apollo::tile_map_images_creator::ImagesCreator;
using apollo::tile_map_images_creator::ImagesCreatorConf;

int main(int argc, char** argv) {
    // init cyber framework
    apollo::cyber::Init(argv[0]);
    // print info log
    FLAGS_alsologtostderr = 1;

    ImagesCreatorConf conf;

    apollo::tile_map_images_creator::ConfigLoader conf_loader;
    if (!conf_loader.LoadConfig(argc, argv, conf)) {
        return -1;
    }

    AINFO << "begin images creator ...";

    ImagesCreator images_creator(conf);
    auto future = images_creator.Start();
    while (!apollo::cyber::IsShutdown()) {
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::timeout) {
            break;
        }
        auto progress = images_creator.GetProgress();
        std::cout << "progress: " << std::fixed << std::setprecision(2) << progress << std::endl;
    }
    if (apollo::cyber::IsShutdown()) {
        return -1;
    }
    auto result = future.get();
    if (!result.succeeded) {
        AERROR << "images create failed: " << result.message;
        return -1;
    }
    AINFO << "images create succeeded, output dir: " << result.output_dir;

    return 0;
}
