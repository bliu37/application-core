load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "grid_matrix",
    srcs = [
        "grid_cell_single.cpp",
        "grid_cell.cpp",
        "grid_matrix.cpp",
        "grid_option.cpp",
    ],
    hdrs = [
        "grid_cell_single.h",
        "grid_cell.h",
        "grid_matrix.h",
        "grid_option.h",
    ],
    copts = COPTS,
    deps = [
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@boost",
        "@com_github_google_glog//:glog",
        "//modules/map_creator/tile_map_images_creator/proto:images_creator_conf_cc_proto",
        "//modules/map_creator/tile_map_images_creator/common",
    ],
)

cc_library(
    name = "grid_map",
    srcs = [
        "grid_map.cpp",
    ],
    hdrs = [
        "grid_map.h",
        "LRU_cache.h",
    ],
    copts = COPTS,
    deps = [
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@boost",
        "@com_github_google_glog//:glog",
        ":grid_matrix",
        "//modules/map_creator/tile_map_images_creator/common",
        "//modules/map_creator/tile_map_images_creator/image_file_generator"
    ],
)


apollo_package()

cpplint()
