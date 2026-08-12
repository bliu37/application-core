load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "image_file_generator",
    srcs = [
        "image_file_generator.cpp",
        "intensity_convertor.cpp",
        "vertical_segmentation.cpp",
    ],
    hdrs = [
        "image_file_generator.h",
        "intensity_convertor.h",
        "vertical_segmentation.h",
    ],
    copts = COPTS,
    deps = [
        "//modules/map_creator/tile_map_images_creator/proto:images_creator_conf_cc_proto",
        "//modules/map_creator/tile_map_images_creator/grid_map:grid_matrix",
        "@boost",
        "@com_github_google_glog//:glog",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
    ],
)

apollo_package()

cpplint()
