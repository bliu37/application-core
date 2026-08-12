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

#include "modules/map_creator/tile_map_images_creator/common/utils.h"

#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace apollo {
namespace tile_map_images_creator {

namespace fs = boost::filesystem;
using json = nlohmann::json;

const char* TILE_JSON_FILE = "tiles.json";

std::string GetTilesJsonFile(const std::string& dirname) {
    if (dirname.back() == '/') {
        return dirname + TILE_JSON_FILE;
    }
    return dirname + '/' + TILE_JSON_FILE;
}

bool GenerateTilesJson(
        const std::string& dirname,
        json* data,
        std::string* message,
        double resolution,
        int base_level) {
    if (!fs::is_directory(dirname)) {
        if (message != nullptr) {
            *message = dirname + " not exists.";
        }
        return false;
    }
    data->clear();
    (*data)["center"] = json::object();
    auto& center = (*data)["center"];
    (*data)["tiles"] = json::object();
    auto& tiles = (*data)["tiles"];

    const std::regex tile_level_re(".*/([0-9])");
    for (auto& level_iter : fs::directory_iterator(dirname)) {
        if (!fs::is_directory(level_iter.path())) {
            continue;
        }
        std::smatch level_match;
        if (!std::regex_match(level_iter.path().string(), level_match, tile_level_re)) {
            continue;
        }
        std::string level = level_match[1].str();
        if (!tiles.contains(level)) {
            tiles[level] = json::array();
        }
        json& level_data = tiles[level];

        // sort by offset_y, offset_x
        std::set<std::string> image_path_set;
        for (auto& entry : fs::recursive_directory_iterator(level_iter.path())) {
            if (fs::is_directory(entry.path())) {
                continue;
            }
            image_path_set.insert(entry.path().string());
        }
        const std::regex tile_image_re(".*/([0-9-]+)/([0-9-]+).png");
        for (auto& image_path : image_path_set) {
            std::smatch image_match;
            if (!std::regex_match(image_path, image_match, tile_image_re)) {
                continue;
            }
            std::string offset_y = image_match[1];
            std::string offset_x = image_match[2];
            level_data.push_back({{"offset_x", offset_x}, {"offset_y", offset_y}});
        }
    }

    if (tiles.empty()) {
        if (message != nullptr) {
            *message = "no tiles info found";
        }
        return false;
    }
    if (!tiles.contains(std::to_string(base_level))) {
        if (message != nullptr) {
            *message = (boost::format("base level %d not found") % base_level).str();
        }
        return false;
    }

    // get center tile coordinate
    const json& level_data = tiles[std::to_string(base_level)];
    const json& center_tile = level_data[level_data.size() / 2];
    int scope = 1024 * resolution;
    std::string offset_x = center_tile["offset_x"];
    std::string offset_y = center_tile["offset_y"];
    center["x"] = std::stoi(offset_x) * scope;
    center["y"] = std::stoi(offset_y) * scope;

    return true;
}

bool DumpTilesJson(const nlohmann::json& data, const std::string& dirname, std::string* message) {
    if (!fs::is_directory(dirname)) {
        if (message != nullptr) {
            *message = dirname + " not exists.";
        }
        return false;
    }

    auto tiles_json_file = GetTilesJsonFile(dirname);
    std::ofstream f(tiles_json_file);
    if (!f.good()) {
        if (message != nullptr) {
            *message = "create " + tiles_json_file + " failed.";
            return false;
        }
    }
    f << std::setw(4) << data << std::endl;
    f.close();
    return true;
}

bool DumpTilesJson(const std::string& dirname, std::string* message, double resolution, int base_level) {
    json data;
    if (!GenerateTilesJson(dirname, &data, message, resolution, base_level)) {
        return false;
    }
    return DumpTilesJson(data, dirname, message);
}

bool LoadTilesJson(const std::string& dirname, nlohmann::json* data, std::string* message) {
    auto tiles_json_file = GetTilesJsonFile(dirname);
    if (!fs::is_regular_file(tiles_json_file)) {
        if (message != nullptr) {
            *message = tiles_json_file + " not found.";
        }
        return false;
    }
    std::ifstream f(tiles_json_file);
    if (!f.good()) {
        if (message != nullptr) {
            *message = "open " + tiles_json_file + " failed.";
            return false;
        }
    }
    try {
        *data = json::parse(f);
    } catch (const json::parse_error& error) {
        f.close();
        if (message != nullptr) {
            *message = (boost::format("parse tiles.json error: %s") % error.what()).str();
            return false;
        }
    }
    f.close();
    return true;
}

uint64_t GetNanosecondTimestampFromSecondTimestamp(double second_timestamp) {
    unsigned long long ll_i = (unsigned long long)second_timestamp;
    unsigned long long ll_f = (second_timestamp - ll_i) * 1e9;
    return ll_i * 1000000000LL + ll_f;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
