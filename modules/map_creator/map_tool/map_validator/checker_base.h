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

#include <memory>

#include "errcode_factory.h"

#include "modules/common_msgs/map_msgs/map.pb.h"

#include "modules/map/hdmap/hdmap.h"

namespace apollo {
namespace map_tool {

class CheckerBase {
public:
    CheckerBase() {}

    virtual ~CheckerBase() = default;

    CheckerBase(
            const std::shared_ptr<apollo::hdmap::Map>& hdmap_message,
            const std::shared_ptr<apollo::hdmap::HDMap>& hdmap_instance) noexcept {
        hdmap_message_ = hdmap_message;
        hdmap_instance_ = hdmap_instance;
    }

    virtual bool Check(std::vector<ErrorMessage>& err_messages) = 0;

    bool AddNewErrorMessage(std::vector<ErrorMessage>& err_messages, ItemCode code, const std::string& element_id) {
        ErrorMessage message;
        RETURN_VAL_IF(!this->errcode_factory_->GetDefaultMessage(code, message), false);
        message.set_element_id(element_id);
        err_messages.emplace_back(std::move(message));
        return true;
    }

protected:
    ErrcodeFactory* errcode_factory_ = ErrcodeFactory::Instance();

    std::shared_ptr<apollo::hdmap::Map> hdmap_message_;
    std::shared_ptr<apollo::hdmap::HDMap> hdmap_instance_;
};
}  // namespace map_tool
}  // namespace apollo
