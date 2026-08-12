load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"map_tool\\"']

cc_library(
    name = "map_convert_tool_lib",
    srcs = [
        "parser/lane_parser.cc",
        "parser/road_parser.cc",
        "parser/crosswalk_parser.cc",
        "parser/junction_parser.cc",
        "parser/speed_bump_parser.cc",
        "parser/map_parser.cc",
        "parser/parking_space_parser.cc",
        "parser/overlap_parser.cc",
        "parser/traffic_signal_parser.cc",
        "parser/barrier_gate_parser.cc",
        "common/map_tool_gflags.cc",
        "utils/util.cc",
    ], 
    hdrs = [
        "common/common.h",
        "parser/lane_parser.h",
        "parser/road_parser.h",
        "parser/crosswalk_parser.h",
        "parser/junction_parser.h",
        "parser/speed_bump_parser.h",
        "parser/map_parser.h",
        "parser/parking_space_parser.h",
        "parser/overlap_parser.h",
        "parser/traffic_signal_parser.h",
        "parser/barrier_gate_parser.h",
        "common/map_tool_gflags.h",
        "utils/util.h",
    ],
    deps = [
        "//modules/map_creator/map_tool/map_validator",
        "//modules/map_creator/map_tool/proto:editor_map_cc_proto",
        "//modules/map_creator/map_tool/proto:map_validator_cc_proto",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Smap_msgs_Cmap_proto",
        "@common//:common",
        "@routing//:modules_Srouting_Capollo_routing",
        "@routing//:routing",
        "//modules/map_creator/tile_map_images_creator/grid_map:grid_map",
        "//modules/map_creator/tile_map_images_creator/common:common",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@common//:modules_Scommon_Smath_Cmath",
        "@map//:modules_Smap_Capollo_map",
        "@map//:map",
        "@eigen",
        "@com_google_absl//:absl",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ]
)

cc_library(
    name = "map_convert_tool",
    srcs = [
        "converter.cc",
        "converter.h",
        
    ],
    copts = COPTS,
    deps = [
        ":map_convert_tool_lib",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "editor_map_converter",
    srcs = [
        "editor_map_cli_main.cc",
    ],
    copts = COPTS,
    deps = [
        ":map_convert_tool",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@routing//:modules_Srouting_Capollo_routing",
        "@routing//:routing",
        "@eigen",
        "@com_google_absl//:absl",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
    ],
)

apollo_package()

cpplint()
