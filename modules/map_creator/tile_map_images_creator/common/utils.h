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

#include <string>

#include <nlohmann/json.hpp>

namespace apollo {
namespace tile_map_images_creator {

/**
 * @brief Generate Tiles Json from images directory
 *
 * @param dirname: The images path
 * @param data: The output tiles json object
 * @param message: output error message
 *
 * @return return true if generate succeeded.
 */
bool GenerateTilesJson(
        const std::string& dirname,
        nlohmann::json* data,
        std::string* message = nullptr,
        double resolution = 0.03125,
        int base_level = 4);

/**
 * @brief Write tiles.json file
 *
 * @param data: The input json data
 * @param dirname: The images dirname
 * @param message: output error message
 *
 * @return return true if write succeeded.
 */
bool DumpTilesJson(const nlohmann::json& data, const std::string& dirname, std::string* message = nullptr);

/**
 * @brief Generate tile json and write tiles.json file
 *
 * @param dirname: The images dirname
 * @param message: output error message
 *
 * @return return true if write succeeded.
 */
bool DumpTilesJson(
        const std::string& dirname,
        std::string* message = nullptr,
        double resolution = 0.03125,
        int base_level = 4);

/**
 * @brief Load json data from tiles.json file
 *
 * @param dirname: The images dirname
 * @param data: The output tiles json object
 * @param message: output error message
 *
 * @return return true if write succeeded.
 */
bool LoadTilesJson(const std::string& dirname, nlohmann::json* data, std::string* message = nullptr);

/**
 * @brief Convert timestamps in seconds to timestamps in nanoseconds. Solve accuracy error during conversion process.
 * @param second_timestamp timestamp in second
 * @return return timestamp in nanosecond
 */
uint64_t GetNanosecondTimestampFromSecondTimestamp(double second_timestamp);

template <typename T>
size_t WriteMessageAndShift(unsigned char*& write_addr, T element) {
    size_t element_size = sizeof(T) / sizeof(unsigned char);
    T* ele_addr = reinterpret_cast<T*>(write_addr);
    *ele_addr = element;
    write_addr += element_size;
    return element_size;
}

template <typename T>
size_t LoadMessageAndShift(unsigned char*& read_addr, T& element) {
    size_t element_size = sizeof(T) / sizeof(unsigned char);
    T* ele_addr = reinterpret_cast<T*>(read_addr);
    element = *ele_addr;
    read_addr += element_size;
    return element_size;
}

}  // namespace tile_map_images_creator
}  // namespace apollo