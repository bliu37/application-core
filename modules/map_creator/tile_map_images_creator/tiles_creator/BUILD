load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "tiles_creator",
    srcs = ["tiles_creator.cc"],
    hdrs = [
        "tiles_creator.h"
    ],
    copts = COPTS,
    deps = [
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "//modules/map_creator/tile_map_images_creator/common",
        "@boost",
        "@com_github_google_glog//:glog",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

apollo_package()

cpplint()
