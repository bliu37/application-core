load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"map_tool\\"']

cc_library(
    name = "map_validator",
    srcs = [
        "map_validator.cpp",
        "checker_lane.cpp",
        "checker_overlap.cpp",
        "checker_parking_space_size.cpp",
        "errcode_factory.cpp",
    ],
    hdrs = [
        "map_validator.h",
        "checker_lane.h",
        "checker_overlap.h",
        "checker_parking_space_size.h",
        "checker_base.h",
        "errcode_factory.h",
    ],
    copts = COPTS,
    deps = [
        "//modules/map_creator/map_tool/proto:editor_map_cc_proto",
        "//modules/map_creator/map_tool/proto:map_validator_cc_proto",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Smap_msgs_Cmap_proto",
        "@common//:common",
        "@common//:modules_Scommon_Smath_Cmath",
        "@map//:modules_Smap_Capollo_map",
        "@map//:map",
        "@eigen",
        "@boost",
        "@com_google_absl//:absl",
        "@com_github_google_glog//:glog",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
    ],
)

# cc_test(
#     name = "map_validator_test",
#     srcs = ["map_validator_test.cpp"],
#     deps = [
#         ":map_validator",
#     ]
# )

apollo_package()

cpplint()
