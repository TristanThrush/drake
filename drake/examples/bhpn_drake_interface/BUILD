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
    srcs = ["src/lcm_utils/robot_state_lcm.cc"],
    hdrs = ["src/lcm_utils/robot_state_lcm.h"],
    deps = [
        "//drake/lcmtypes:robot_state",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "plan_status_lcm",
    srcs = ["src/lcm_utils/plan_status_lcm.cc"],
    hdrs = ["src/lcm_utils/plan_status_lcm.h"],
    deps = [
        "@lcmtypes_robotlocomotion",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)


drake_cc_library(
    name = "sim_diagram_builder",
    srcs = ["src/simulation_utils/sim_diagram_builder.cc"],
    hdrs = ["src/simulation_utils/sim_diagram_builder.h"],
    deps = [
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/controllers:state_feedback_controller_interface",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "world_sim_tree_builder",
    srcs = ["src/simulation_utils/world_sim_tree_builder.cc"],
    hdrs = ["src/simulation_utils/world_sim_tree_builder.h"],
    deps = [
        "//drake/common:unused",
	"//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
    ],
)

drake_cc_library(
    name = "robot_plan_interpolator",
    srcs = ["src/lcm_utils/robot_plan_interpolator.cc"],
    hdrs = ["src/lcm_utils/robot_plan_interpolator.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework:leaf_system",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_library(
    name = "pr2_fixed_controller",
    srcs = ["src/controllers/pr2_fixed_controller.cc"],
    hdrs = ["src/controllers/pr2_fixed_controller.h"],
    deps = [
        ":robot_state_lcm",
	":robot_plan_interpolator",
        ":sim_diagram_builder",
        ":world_sim_tree_builder",
        "//drake/lcm",
	"//drake/util:lcm_util",
        "//drake/multibody/rigid_body_plant",
	"//drake/multibody/rigid_body_plant:contact_results_to_lcm",
	"//drake/multibody:inverse_kinematics",
        "//drake/systems/analysis:simulator",
	"//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/primitives:constant_vector_source",
	"//drake/systems/primitives:trajectory_source",
	"//drake/systems/primitives:multiplexer",
	"//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:piecewise_polynomial",
	"//drake/common:find_resource",
	"@com_github_gflags_gflags//:gflags",
	"@lcmtypes_robotlocomotion",
    ],

    )

drake_cc_binary(
    name = "simulation",
    srcs = ["src/simulation.cc"],
    deps = [
        ":robot_state_lcm",
	":pr2_fixed_controller",
	":robot_plan_interpolator",
        ":sim_diagram_builder",
        ":world_sim_tree_builder",
        ":plan_status_lcm",
        "//drake/lcm",
	"//drake/util:lcm_util",
        "//drake/multibody/rigid_body_plant",
	"//drake/multibody/rigid_body_plant:contact_results_to_lcm",
	"//drake/multibody:inverse_kinematics",
        "//drake/systems/analysis:simulator",
	"//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/primitives:constant_vector_source",
	"//drake/systems/primitives:trajectory_source",
	"//drake/systems/primitives:multiplexer",
	"//drake/systems/analysis:implicit_euler_integrator",
	"//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:piecewise_polynomial",
	"//drake/common:find_resource",
	"//drake/common:text_logging_gflags",
	"@com_github_gflags_gflags//:gflags",
	"@lcmtypes_robotlocomotion",
    ],
)

cpplint()
