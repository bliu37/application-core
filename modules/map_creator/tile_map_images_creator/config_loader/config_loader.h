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
#include "modules/map_creator/tile_map_images_creator/proto/images_creator_conf.pb.h"
#include "modules/map_creator/tile_map_images_creator/config_loader/cmdline_parser.h"

namespace apollo {
namespace tile_map_images_creator {

class ConfigLoader {
public:
    ConfigLoader();

    bool LoadConfig(int argc, char* argv[], ImagesCreatorConf& conf);

private:
    std::unique_ptr<CmdlineTool> cmdline_tool_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
