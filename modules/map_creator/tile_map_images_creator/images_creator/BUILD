load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "images_creator",
    srcs = [
        "images_creator.cc",
        "images_renderer.cpp",
        "images_frame_producer.cpp",
        "pcd_creator.cpp",
        "transform_workers.cpp"
    ],
    hdrs = [
        "images_creator.h",
        "images_creator_result.h",
        "images_renderer.h",
        "images_frame_producer.h",
        "pcd_creator.h",
        "transform_workers.h"
    ],
    copts = COPTS,
    deps = [
        "//modules/map_creator/tile_map_images_creator/proto:images_creator_conf_cc_proto",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "//modules/map_creator/tile_map_images_creator/matrix_generator",
        "//modules/map_creator/tile_map_images_creator/point_cloud_filter",
        "//modules/map_creator/tile_map_images_creator/common",
        "//modules/map_creator/tile_map_images_creator/tiles_creator",
        "//modules/map_creator/tile_map_images_creator/config_loader",
        "//modules/map_creator/tile_map_images_creator/time_align",
        "//modules/map_creator/tile_map_images_creator/coord_transformer",
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "@boost",
        "@com_github_google_glog//:glog",
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()

cpplint()
