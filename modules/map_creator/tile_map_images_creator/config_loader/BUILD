load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "config_loader",
    srcs = [
        "cmdline_parser.cpp",
        "config_loader.cpp",
    ],
    hdrs = [
        "cmdline_parser.h",
        "config_loader.h",
    ],
    copts = COPTS,
    deps = [
        "//modules/map_creator/tile_map_images_creator/proto:images_creator_conf_cc_proto",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@local_config_pcl//:pcl",
        "@boost",
        "@com_github_google_glog//:glog",
    ],
)

apollo_package()
