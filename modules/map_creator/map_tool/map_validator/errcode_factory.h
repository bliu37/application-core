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
#include <map>
#include "modules/map_creator/map_tool/proto/map_validator.pb.h"
#include "cyber/cyber.h"

namespace apollo {
namespace map_tool {

class ErrcodeFactory {
public:
    bool Init(const std::string& errcode_conf_path);

    bool GetDefaultMessage(ItemCode code, ErrorMessage& default_message);

private:
    std::map<ItemCode, ErrorMessage> errcode_db;

    DECLARE_SINGLETON(ErrcodeFactory);
};

}  // namespace map_tool
}  // namespace apollo
