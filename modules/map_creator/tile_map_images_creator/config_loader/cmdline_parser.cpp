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

#include "modules/map_creator/tile_map_images_creator/config_loader/cmdline_parser.h"

namespace apollo {
namespace tile_map_images_creator {

CmdlineTool::CmdlineTool() {
    args_str_map_["input"] = "";
    args_str_map_["output"] = "";
    args_str_map_["config"] = "";
}

bool CmdlineTool::ParseCmd(const int argc, char* argv[]) {
    std::string usage_message
            = "Usage: tile_map_images_creator [-c configure_file_path | -i input_records_directory | -o "
              "output_images_directory]. Specifying -i and -o options will override the corresponding settings in "
              "configuration file";
    boost::program_options::options_description params_opt(usage_message);

    params_opt.add_options()(
            "input,i",
            boost::program_options::value<std::string>(),
            "specific input records directory, will override the corresponding setting in the configuration file")(
            "output,o",
            boost::program_options::value<std::string>(),
            "specific output images directory, will override the corresponding setting in the configuration file")(
            "config,c",
            boost::program_options::value<std::string>(),
            "specific configure file path, default = "
            "/apollo/modules/map_creator/tile_map_images_creator/conf/image_creator_conf.pb.txt")(
            "help,h", "provide help message");

    boost::program_options::variables_map boost_args;

    try {
        auto options = boost::program_options::parse_command_line(argc, argv, params_opt);
        boost::program_options::store(options, boost_args);
    } catch (const boost::program_options::error& exception) {
        std::cerr << params_opt << std::endl << std::endl;
        std::cerr << "Incorrect command line arguments format." << std::endl;
        return false;
    }

    for (auto it = boost_args.begin(); it != boost_args.end(); ++it) {
        // if more types of arguments need to be handled, add new std::map for distinct type and judge them
        // independently for processing
        std::string arg_key = it->first;
        if (!args_str_map_.count(arg_key)) {
            std::cout << params_opt << std::endl;
            if (arg_key != "help") {
                std::cerr << "Unable to recognize command line key: " << arg_key << std::endl;
            }
            return false;
        } else if (args_str_map_.count(arg_key)) {
            args_str_map_[arg_key] = boost_args[arg_key].as<std::string>();
        }
    }
    return true;
}

void CmdlineTool::OverrideStrOptionIfExist(const std::string& key, std::string& option) {
    if (args_str_map_.count(key) && args_str_map_[key].length() > 0) {
        option = args_str_map_[key];
    }
}

}  // namespace tile_map_images_creator
}  // namespace apollo