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
#include "modules/map_creator/map_tool/converter.h"

namespace apollo {
namespace map_tool {

Status LoadEditorMap(const std::string& filename, EditorMap* data) {
    if (!apollo::cyber::common::GetProtoFromBinaryFile(filename, data)) {
        return Status{Code::ERROR, "Failed to load map from binary file."};
    }
    return Status{Code::OK, ""};
}

Status LoadEditorMapToJson(const std::string& filename, nlohmann::json* data) {
    apollo::map_tool::EditorMap editor_map;
    if (!apollo::cyber::common::GetProtoFromBinaryFile(filename, &editor_map)) {
        return Status{Code::ERROR, "Failed to load editor map from file."};
    }
    std::vector<std::string> json_options;
    json_options.push_back("enums_as_ints");
    json_options.push_back("primitive_fields");
    *data = apollo::map_tool::ProtoToJson(editor_map, json_options);
    return Status{Code::OK, ""};
}

Status SaveEditorMap(const std::string& filename, const EditorMap& data) {
    if (!apollo::cyber::common::SetProtoToBinaryFile(data, filename)) {
        return Status{Code::ERROR, "Failed to save map to binary file."};
    }
    return Status{Code::OK, ""};
}

Status SaveEditorMapFromJson(const std::string& filename, const nlohmann::json& data) {
    apollo::map_tool::EditorMap editor_map;
    if (!apollo::map_tool::GetProtoFromJson(data, &editor_map)) {
        return Status{Code::ERROR, "Failed to save editor map from json."};
    }
    return SaveEditorMap(filename, editor_map);
}

Status ConvertToApolloMap(const std::string& filename, apollo::hdmap::Map* apollo_map) {
    if (!apollo::cyber::common::GetProtoFromFile(filename, apollo_map)) {
        AERROR << "Map file in binary format load error: path = " << FLAGS_routing_map_config_path;
        return Status{Code::ERROR, "Failed to covert editor map from binary file."};
    }
    return Status{Code::OK, ""};
}

Status ConvertToApolloMap(EditorMap& editor_map, apollo::hdmap::Map* apollo_map) {
    if (!EditorMapParser(editor_map, apollo_map)) {
        return Status{Code::ERROR, "Failed to parser editor map."};
    }
    return Status{Code::OK, ""};
}

Status ConvertToApolloMap(const nlohmann::json& data, apollo::hdmap::Map* apollo_map) {
    if (!EditorMapParser(data, apollo_map)) {
        return Status{Code::ERROR, "Failed to parser map json."};
    }
    return Status{Code::OK, ""};
}

Status ReleaseApolloMap(const std::string& filename, apollo::hdmap::Map& apollo_map) {
    std::string base_map_path = filename + "/base_map.bin";
    std::string local_map_path = FLAGS_curr_base_map_dir + "/local_map";
    std::string local_map_dump_path = filename + "/local_map";
    if (!apollo::cyber::common::SetProtoToBinaryFile(apollo_map, base_map_path)) {
        AERROR << "Failed to save hdmap to binary file.";
        return Status{Code::ERROR, "Failed to save hdmap to binary file."};
    }
    std::ofstream base_map_txt_file(filename + "/base_map.txt");
    base_map_txt_file << apollo_map.DebugString();
    base_map_txt_file.close();
    if (apollo::cyber::common::PathExists(local_map_path)) {
        if (!apollo::cyber::common::CopyDir(local_map_path, local_map_dump_path)) {
            AERROR << "Failed to dump location map.";
            return Status{Code::ERROR, "Failed to dump localion map."};
        }
    }
    GenerateSimMap(filename, &apollo_map);
    GenerateRoutingMap(filename);
    return Status{Code::OK, ""};
}

Status ReleaseApolloMap(const std::string& filename, const nlohmann::json& data) {
    apollo::hdmap::Map* map = new apollo::hdmap::Map();
    if (!EditorMapParser(data, map)) {
        return Status{Code::ERROR, "Failed to parser map json."};
    }
    ReleaseApolloMap(filename, *map);
    return Status{Code::OK, ""};
}

bool MapValidityCheck(apollo::hdmap::Map& apollo_map, nlohmann::json* error_message) {
    ValidatorResult result;
    if (!VerifyMapValidity(apollo_map, &result)) {
        AERROR << "Failed to start map validity check.";
        return false;
    }
    if (result.error_message().size() > 0) {
        std::vector<std::string> json_options;
        json_options.push_back("primitive_fields");
        *error_message = apollo::map_tool::ProtoToJson(result, json_options);
        for (const auto& iter : result.error_message()) {
            if (iter.level() == ErrorLevel::Error) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace map_tool
}  // namespace apollo
