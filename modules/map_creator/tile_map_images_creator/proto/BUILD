load("//tools/proto:proto.bzl", "proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "images_creator_conf_proto",
    srcs = ["images_creator_conf.proto"],
)

proto_library(
    name = "slam_pose_result_proto",
    srcs = ["slam_pose_result.proto"],
    deps = [
        "//modules/common_msgs/transform_msgs:transform_proto",
    ],
)

apollo_package()
