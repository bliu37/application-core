load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary", "cc_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
COPTS = ['-DMODULE_NAME=\\"tile_map_images_creator\\"']

# cc_test(
#     name = "check_matrix_save_and_load",
#     srcs = ["check_matrix_save_and_load/check_matrix_save_and_load.cpp"],
#     deps = [
#         "//modules/map_creator/tile_map_images_creator/grid_map",
#         "@com_google_googletest//:gtest_main",
#     ]
# )

apollo_package()
