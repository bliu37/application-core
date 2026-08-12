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

#include "config_loader.h"
#include "cyber/cyber.h"

namespace apollo {
namespace tile_map_images_creator {

ConfigLoader::ConfigLoader() {
    cmdline_tool_ = std::make_unique<CmdlineTool>();
}

bool ConfigLoader::LoadConfig(int argc, char *argv[], ImagesCreatorConf &conf) {
    std::string config_file_path;
    if (!cmdline_tool_->ParseCmd(argc, argv)) {
        return false;
    }

    std::string image_creator_conf_path
            = "/apollo/modules/map_creator/tile_map_images_creator/conf/image_creator_conf.pb.txt";
    cmdline_tool_->OverrideConfigurationFilePathIfExist(image_creator_conf_path);

    if (!apollo::cyber::common::GetProtoFromFile(image_creator_conf_path, &conf)) {
        AERROR << "configure file in protobuf format load error: path = " << image_creator_conf_path;
        return false;
    }

    cmdline_tool_->OverrideInputDirectoryOptionIfExist(*conf.mutable_input_output_conf()->mutable_input_dir());
    cmdline_tool_->OverrideOutputDirectoryOptionIfExist(*conf.mutable_input_output_conf()->mutable_images_output_dir());

    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
