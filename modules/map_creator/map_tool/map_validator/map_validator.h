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
#include <string>
#include "modules/common_msgs/map_msgs/map.pb.h"
#include "modules/map_creator/map_tool/proto/map_validator.pb.h"
#include "cyber/cyber.h"

#include "modules/map_creator/map_tool/map_validator/errcode_factory.h"
#include "modules/map_creator/map_tool/map_validator/checker_lane.h"
#include "modules/map_creator/map_tool/map_validator/checker_overlap.h"
#include "modules/map_creator/map_tool/map_validator/checker_parking_space_size.h"

namespace apollo {
namespace map_tool {

/**
 * @brief Final verification for hdmap before publishing.
 */
class MapValidator {
public:
    MapValidator();

    /**
     * @brief Validator init, need several protobuf conf files, and binary format of apollo hdmap file
     * @param checker_conf_path path of checker conf, describe detection items.
     * @param errcode_conf_path path of errcode conf, describe errcode default level and title.
     * @param map_bin_path path of apollo hdmap, format in binary protobuf.
     * @return init result, true as OK, false as Fail.
     */
    bool Init(
            const std::string& checker_conf_path,
            const std::string& errcode_conf_path,
            const std::string& map_bin_path);

    /**
     * @brief Validator init, need several protobuf conf files, and binary format of apollo hdmap file
     * @param checker_conf_path path of checker conf, describe detection items.
     * @param errcode_conf_path path of errcode conf, describe errcode default level and title.
     * @param map. an instance of apollo hdmap.
     * @return init result, true as OK, false as Fail.
     */
    bool Init(
            const std::string& checker_conf_path,
            const std::string& errcode_conf_path,
            const apollo::hdmap::Map& map);

    /**
     * @brief Run all checker for map validation.
     * @return result. error messages return
     * @return true for result is reliable, false for some errors occur.
     */
    bool Check(ValidatorResult& result);

private:
    std::shared_ptr<apollo::hdmap::Map> hdmap_message_;
    std::shared_ptr<apollo::hdmap::HDMap> hdmap_instance_;

    std::unique_ptr<CheckerLane> checker_lane_;
    std::unique_ptr<CheckerOverlap> checker_overlap_;
    std::unique_ptr<CheckerParkingSpaceSize> checker_parking_space_size_;

    MapValidatorConfig config_;
};

}  // namespace map_tool
}  // namespace apollo