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
#include <iostream>
#include "boost/program_options.hpp"
#include "boost/throw_exception.hpp"

namespace apollo {
namespace tile_map_images_creator {

class CmdlineTool {
public:
    CmdlineTool();
    bool ParseCmd(const int argc, char* argv[]);

    void OverrideInputDirectoryOptionIfExist(std::string& option) {
        OverrideStrOptionIfExist("input", option);
    }

    void OverrideOutputDirectoryOptionIfExist(std::string& option) {
        OverrideStrOptionIfExist("output", option);
    }

    void OverrideConfigurationFilePathIfExist(std::string& option) {
        OverrideStrOptionIfExist("config", option);
    }

private:
    void OverrideStrOptionIfExist(const std::string& key, std::string& option);

    std::map<std::string, std::string> args_str_map_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
