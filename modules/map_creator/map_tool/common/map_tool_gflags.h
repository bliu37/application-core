/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "gflags/gflags.h"

DECLARE_string(routing_map_config_path);

DECLARE_double(overlap_error_range);

DECLARE_double(curve_sampling_step_size);

DECLARE_double(curve_sampling_rate);

DECLARE_string(default_checker_config_path);

DECLARE_string(default_errcode_config_path);

DECLARE_double(default_associated_distance_parking_space_lane);

DECLARE_string(curr_base_map_dir);