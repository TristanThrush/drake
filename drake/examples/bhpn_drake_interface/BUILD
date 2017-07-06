# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

package(
    default_visibility = [":__subpackages__"],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
	"**/*.dae",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "robot_state_lcm",
    srcs = ["lcm_utils/robot_state_lcm.cc"],
    hdrs = ["lcm_utils/robot_state_lcm.h"],
    linkstatic = 1,
    deps = [
        "//drake/lcmtypes:robot_state",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "sim_diagram_builder",
    srcs = ["simulation_utils/sim_diagram_builder.cc"],
    hdrs = ["simulation_utils/sim_diagram_builder.h"],
    linkstatic = 1,
    deps = [
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/controllers:state_feedback_controller_interface",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "world_sim_tree_builder",
    srcs = ["simulation_utils/world_sim_tree_builder.cc"],
    hdrs = ["simulation_utils/world_sim_tree_builder.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:unused",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
    ],
)

drake_cc_binary(
    name = "simulation",
    srcs = ["simulation.cc"],
    linkstatic = 1,
    deps = [
        ":robot_state_lcm",
        ":sim_diagram_builder",
        ":world_sim_tree_builder",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
	"//drake/multibody:inverse_kinematics",
        "//drake/systems/analysis:simulator",
	"//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/primitives:constant_vector_source",
	"//drake/systems/primitives:trajectory_source",
	"//drake/systems/primitives:multiplexer",
	"//drake/common/trajectories:piecewise_polynomial_trajectory",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
