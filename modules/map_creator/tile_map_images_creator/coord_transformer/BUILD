load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "coord_transformer",
    srcs = [
        "static_transformer.cpp",
        "frame_transformer.cpp",
    ],
    hdrs = [
        "static_transformer.h",
        "frame_transformer.h",
    ],
    copts = COPTS,
    deps = [
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "//modules/map_creator/tile_map_images_creator/time_align",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@transform//:modules_Stransform_Capollo_transform",
        "@transform//:transform",
        "@tf2//:tf2",
        "//modules/map_creator/tile_map_images_creator/common",
        "@eigen",
        "@com_github_google_glog//:glog",
    ]
)

cc_test(
    name = "static_transformer_test",
    size = "small",
    srcs = [
        "static_transformer_test.cpp",
    ],
    deps = [
        ":coord_transformer",
        "@boost",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ]
)

cc_test(
    name = "frame_transformer_test",
    size = "small",
    srcs = [
        "frame_transformer_test.cpp",
    ],
    deps = [
        ":coord_transformer",
        "//modules/map_creator/tile_map_images_creator/time_align",
        "@boost",
        "@com_github_google_glog//:glog",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
