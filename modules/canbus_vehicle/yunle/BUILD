load(
    "//tools:apollo_package.bzl",
    "apollo_cc_binary",
    "apollo_cc_library",
    "apollo_cc_test",
    "apollo_component",
    "apollo_package",
)
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "yunle_feedback_protocol",
    srcs = ["yunle_protocol.cc"],
    hdrs = ["yunle_protocol.h"],
)

apollo_cc_library(
    name = "yunle_control_protocol",
    srcs = ["yunle_control_protocol.cc"],
    hdrs = ["yunle_control_protocol.h"],
)

apollo_component(
    name = "libyunle_chassis_receiver_component.so",
    srcs = ["yunle_chassis_receiver_component.cc"],
    copts = ['-DMODULE_NAME=\\"yunle_chassis_receiver\\"'],
    deps = [
        ":yunle_control_protocol",
        ":yunle_feedback_protocol",
        "//modules/canbus_vehicle/yunle/proto:yunle_chassis_receiver_cc_proto",
        "@common//:modules_Scommon_Sutil_Cmessage_util",
        "@common-msgs//:modules_Scommon_msgs_Schassis_msgs_Cchassis_proto",
        "@common-msgs//:modules_Scommon_msgs_Scontrol_msgs_Ccontrol_cmd_proto",
        "@cyber//:cyber_Ccyber",
    ],
)

apollo_cc_test(
    name = "yunle_protocol_test",
    size = "small",
    srcs = ["yunle_protocol_test.cc"],
    deps = [
        ":yunle_feedback_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "yunle_control_protocol_test",
    size = "small",
    srcs = ["yunle_control_protocol_test.cc"],
    deps = [
        ":yunle_control_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "yunle_control_protocol_self_test",
    srcs = ["yunle_control_protocol_self_test.cc"],
    deps = [":yunle_control_protocol"],
)

apollo_package()

cpplint()
