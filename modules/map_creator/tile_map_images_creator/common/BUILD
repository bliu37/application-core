load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "common",
    srcs = [
        "utils.cc",
        "image_creator_flags.cc",
    ],
    hdrs = [
        "macro.h",
        "utils.h",
        "frame.h",
        "image_creator_flags.h",
    ],
    copts = COPTS,
    deps = [
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@boost",
        "@com_github_nlohmann_json//:json",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

apollo_package()

cpplint()
