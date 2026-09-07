load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_library(
    name = "pclomp_ndt_omp",
    srcs = [
        "third_party/ndt_omp/src/pclomp/gicp_omp.cpp",
        "third_party/ndt_omp/src/pclomp/ndt_omp.cpp",
        "third_party/ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp",
    ],
    hdrs = glob([
        "third_party/ndt_omp/include/pclomp/*.h",
        "third_party/ndt_omp/include/pclomp/*.hpp",
    ]),
    copts = [
        "-fopenmp",
        "-faligned-new",
        "-msse",
        "-msse2",
        "-msse3",
        "-msse4",
        "-msse4.1",
        "-msse4.2",
    ],
    includes = ["third_party/ndt_omp/include"],
    linkopts = ["-lgomp"],
    deps = [
        "@boost",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_component(
    name = "libyunle_dreamview_bridge_component.so",
    srcs = [
        "indoor_empty_prediction_publisher.cc",
        "indoor_ndt_input_bridge.cc",
        "indoor_ndt_pose_stabilizer.cc",
        "indoor_test_tf_publisher.cc",
        "localization_time_bridge.cc",
        "n100_imu_driver.cc",
    ],
    copts = ['-DMODULE_NAME=\\"yunle_dreamview_bridge\\"'],
    deps = [
        "@common//:modules_Scommon_Sutil_Cmessage_util",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Cgps_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Cimu_proto",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cimu_proto",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@common-msgs//:modules_Scommon_msgs_Sprediction_msgs_Cprediction_obstacle_proto",
        "@common-msgs//:modules_Scommon_msgs_Stransform_msgs_Ctransform_proto",
        "@cyber//:cyber_Ccyber",
        "@loam-velodyne-indoor//:modules_Sloam_velodyne_indoor_Sproto_Codometry_proto",
    ],
)

apollo_component(
    name = "libyunle_indoor_pclomp_ndt_localization_component.so",
    srcs = ["indoor_pclomp_ndt_localizer.cc"],
    copts = [
        '-DMODULE_NAME=\\"yunle_indoor_pclomp_ndt_localization\\"',
        "-faligned-new",
        "-fopenmp",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":pclomp_ndt_omp",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Cgps_proto",
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@common-msgs//:modules_Scommon_msgs_Ssensor_msgs_Cpointcloud_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@cyber//:cyber_Ccyber",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_component(
    name = "libyunle_indoor_planning_heading_adapter_component.so",
    srcs = ["indoor_planning_heading_adapter.cc"],
    copts = ['-DMODULE_NAME=\\"yunle_dreamview_bridge\\"'],
    deps = [
        "@common-msgs//:modules_Scommon_msgs_Slocalization_msgs_Clocalization_proto",
        "@common-msgs//:modules_Scommon_msgs_Srouting_msgs_Crouting_proto",
        "@cyber//:cyber_Ccyber",
    ],
)

apollo_component(
    name = "libyunle_indoor_planning_command_adapter_component.so",
    srcs = ["indoor_planning_command_adapter.cc"],
    copts = ['-DMODULE_NAME=\\"yunle_dreamview_bridge\\"'],
    deps = [
        "@common-msgs//:modules_Scommon_msgs_Splanning_msgs_Cplanning_command_proto",
        "@cyber//:cyber_Ccyber",
    ],
)

apollo_package()

cpplint()
