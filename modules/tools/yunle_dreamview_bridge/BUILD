load("//tools:apollo_package.bzl", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "libyunle_dreamview_bridge_component.so",
    srcs = ["localization_time_bridge.cc"],
    copts = ['-DMODULE_NAME=\\"yunle_dreamview_bridge\\"'],
    deps = [
        "@common//:modules_Scommon_Sutil_Cmessage_util",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@cyber//:cyber_Ccyber",
    ],
)

apollo_package()

cpplint()
