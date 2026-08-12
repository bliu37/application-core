load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

cc_library(
    name = "time_align",
    srcs = [
        "multi_source_frame_loader.cpp",
        "single_source_frame_loader.cpp",
        "record_align_adapter.cpp",
    ],
    hdrs = [
        "frame_loader.h",
        "multi_source_frame_loader.h",
        "single_source_frame_loader.h",
        "time_align_cache.hpp",
        "record_align_adapter.h",
        "vector_align_adapter.hpp",
    ],
    copts = COPTS,
    deps = [
        "@cyber//:cyber",
        "@fastrtps",
        "@cyber//:cyber_Ccyber",
        "//modules/map_creator/tile_map_images_creator/common",
        "//modules/map_creator/tile_map_images_creator/proto:slam_pose_result_cc_proto",
        "@boost",
        "@com_github_google_glog//:glog",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
    ]
)

cc_test(
    name = "time_align_cache_test",
    size="small",
    srcs = [
        "time_align_cache_test.cpp",
    ],
    deps = [
        ":time_align",
        "@boost",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "record_align_adapter_test",
    size = "small",
    srcs = [
        "record_align_adapter_test.cpp",
    ],
    deps = [
        ":time_align",
        "@boost",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "vector_align_adapter_test",
    size = "small",
    srcs = [
        "vector_align_adapter_test.cpp",
    ],
    deps = [
        ":time_align",
        "@boost",
        "@com_github_google_glog//:glog",
        "@common-msgs//:common-msgs",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "frame_loader_test",
    size = "small",
    srcs = [
        "frame_loader_test.cpp",
    ],
    deps = [
        ":time_align",
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
