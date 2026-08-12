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

#include "modules/map_creator/tile_map_images_creator/common/frame.h"

namespace apollo {
namespace tile_map_images_creator {

/**
 * @brief the interface for frame loader
 */
class FrameLoaderInterface {
public:
    virtual ~FrameLoaderInterface() = default;
    virtual bool Init() = 0;
    virtual bool LoadNextFrame(ImageCreatorFrame& frame) = 0;
};

}  // namespace tile_map_images_creator
}  // namespace apollo
