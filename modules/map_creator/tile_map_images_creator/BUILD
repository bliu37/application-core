load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_binary(
    name = "tile_map_images_creator",
    srcs = ["main.cc"],
    linkstatic = True,
    copts = COPTS,
    deps = [
        "//modules/map_creator/tile_map_images_creator/proto:images_creator_conf_cc_proto",
        "//modules/map_creator/tile_map_images_creator/images_creator",
        "//modules/map_creator/tile_map_images_creator/tiles_creator",
        "//modules/map_creator/tile_map_images_creator/matrix_generator",
        "//modules/map_creator/tile_map_images_creator/grid_map",
        "//modules/map_creator/tile_map_images_creator/time_align",
        "//modules/map_creator/tile_map_images_creator/coord_transformer",
        "@boost",
        "@com_google_absl//:absl",
        "@com_github_google_glog//:glog",
        "@local_config_pcl//:pcl",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*",
    ]),
)

apollo_package()

cpplint()
