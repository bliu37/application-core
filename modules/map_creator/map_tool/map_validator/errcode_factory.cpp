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
 *****************************************************************************/ \
#include "errcode_factory.h"

namespace apollo {
namespace map_tool {

ErrcodeFactory::ErrcodeFactory() = default;

bool ErrcodeFactory::Init(const std::string& errcode_conf_path) {
    ValidatorResult default_message;
    if (!apollo::cyber::common::GetProtoFromASCIIFile(errcode_conf_path, &default_message)) {
        AERROR << "load errcode config file error, path = " << errcode_conf_path;
        return false;
    }

    for (int i = 0; i < default_message.error_message_size(); ++i) {
        ItemCode code = default_message.error_message(i).item_code();
        errcode_db[code] = default_message.error_message(i);
    }
    AINFO << "Errcode Init Complete, default error message num = " << errcode_db.size();
    return true;
}

bool ErrcodeFactory::GetDefaultMessage(ItemCode code, ErrorMessage& default_message) {
    if (!errcode_db.count(code)) {
        default_message = ErrorMessage();
        default_message.set_item_code(code);
        default_message.set_level(Normal);
        default_message.set_title("");
        AERROR << "cannot get default message for code = " << code;
        return false;
    }
    default_message = errcode_db[code];
    return true;
}

}  // namespace map_tool
}  // namespace apollo
