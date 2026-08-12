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
#include "modules/map_creator/map_tool/common/map_tool_gflags.h"

DEFINE_string(routing_map_config_path, "/apollo/modules/routing/conf/routing_config.pb.txt", "routing config path");

DEFINE_double(overlap_error_range, 0.003, "Error range for overlapping area calculations.");

DEFINE_double(curve_sampling_step_size, 0.3, "Interpolation step size for Bezier curves.");

DEFINE_double(
        curve_sampling_rate,
        0.3,
        "The sampling rate of the curve. The number of points in a curve is 1/curve_sampling_rate.");
DEFINE_string(
        default_checker_config_path,
        "/apollo/modules/map_creator/map_tool/conf/map_validator_checker_conf.pb.txt",
        "The default configuration path of map quality inspection rules.");

DEFINE_string(
        default_errcode_config_path,
        "/apollo/modules/map_creator/map_tool/conf/map_validator_errcode_conf.pb.txt",
        "Default configuration path for map quality inspection error reporting format.");

DEFINE_double(
        default_associated_distance_parking_space_lane,
        0.5,
        "The associated distance between the parking space and the road.");

DEFINE_string(curr_base_map_dir, "", "curr base map dir path");