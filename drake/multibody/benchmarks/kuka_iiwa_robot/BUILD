# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = [":__subpackages__"])

drake_cc_library(
    name = "kuka_iiwa_robot_library",
    testonly = 1,
    srcs = [
        "test/MG_kuka_iiwa_robot.cc",
        "test/MG_kuka_iiwa_robot_auto_generated.cc",
    ],
    hdrs = [
        "drake_kuka_iiwa_robot.h",
        "test/MG_kuka_iiwa_robot.h",
        "test/MG_kuka_iiwa_robot_auto_generated.h",
    ],
    deps = [
        "//drake/common:autodiff",
        "//drake/common:extract_double",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "MG_kuka_iiwa_robot_test",
    size = "small",
    deps = [
        ":kuka_iiwa_robot_library",
    ],
)

drake_cc_googletest(
    name = "kuka_iiwa_robot_kinematics_test",
    size = "small",
    deps = [
        ":kuka_iiwa_robot_library",
        "//drake/math:geometric_transform",
        "//drake/multibody/multibody_tree",
        "//drake/multibody/multibody_tree:bodies_and_frames",
        "//drake/multibody/multibody_tree:mobilizers_and_body_nodes",
    ],
)

add_lint_tests()
