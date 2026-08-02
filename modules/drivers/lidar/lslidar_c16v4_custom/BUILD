load("//tools:apollo_package.bzl", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "liblslidar_c16v4_custom_component.so",
    srcs = ["lslidar_component.cpp"],
    copts = ['-DMODULE_NAME=\\"lslidar_c16v4_custom\\"'],
    deps = [
        "@common//:modules_Scommon_Sutil_Cmessage_util",
        "@cyber//:cyber_Ccyber",
        "@drivers-lidar//:modules_Sdrivers_Slidar_Scommon_Clidar_common",
        "@drivers-lidar//:modules_Sdrivers_Slidar_Slslidar_Sdriver_Cdriver",
        "@drivers-lidar//:modules_Sdrivers_Slidar_Slslidar_Sparser_Cconvert",
        "@drivers-lidar//:modules_Sdrivers_Slidar_Slslidar_Sproto_Cconfig_proto",
        "@drivers-lidar//:modules_Sdrivers_Slidar_Slslidar_Sproto_Clslidar_proto",
    ],
)

apollo_package()

cpplint()
