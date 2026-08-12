load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "point_cloud_filter",
    srcs = [
        "point_cloud_preprocessing_filter.cpp"
    ],
    hdrs = [
        "point_cloud_preprocessing_filter.h"
    ],
    copts = COPTS,
    deps = [
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "//modules/map_creator/tile_map_images_creator/proto:images_creator_conf_cc_proto",
        "//modules/map_creator/tile_map_images_creator/common",
        "@boost",
        "@com_github_google_glog//:glog",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
    ],
)

apollo_package()

cpplint()
