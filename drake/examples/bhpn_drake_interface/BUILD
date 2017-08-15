# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)
load("//tools:install_data.bzl", "install_data")

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "robot_state_lcm",
    srcs = ["src/utils/robot_state_lcm.cc"],
    hdrs = ["src/utils/robot_state_lcm.h"],
    deps = [
        "@lcmtypes_bot2_core",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "plan_status_lcm",
    srcs = ["src/utils/plan_status_lcm.cc"],
    hdrs = ["src/utils/plan_status_lcm.h"],
    deps = [
        "@lcmtypes_robotlocomotion",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "world_sim_tree_builder",
    srcs = ["src/utils/world_sim_tree_builder.cc"],
    hdrs = ["src/utils/world_sim_tree_builder.h"],
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
    name = "valkyrie_world_diagram",
    hdrs = ["src/simulators/valkyrie/valkyrie_world_diagram.h"],
    deps = [
        "//drake/examples/valkyrie:actuator_effort_to_rigid_body_plant_input_converter",
        "//drake/examples/valkyrie:robot_command_to_desired_effort_converter",
        "//drake/examples/valkyrie:valkyrie_constants",
	"//drake/examples/kuka_iiwa_arm:oracular_state_estimator",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/lcm:lcmt_drake_signal_translator",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:pass_through",
    ],
)

drake_cc_library(
    name = "robot_plan_interpolator",
    srcs = ["src/utils/robot_plan_interpolator.cc"],
    hdrs = ["src/utils/robot_plan_interpolator.h"],
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
    name = "bdisc_parser",
    hdrs = ["src/utils/bdisc_parser.h"],
    )

drake_cc_binary(
    name = "pr2_simulator",
    srcs = ["src/simulators/pr2/pr2_simulator.cc"],
    deps = [
        ":robot_state_lcm",
	":robot_plan_interpolator",
        ":world_sim_tree_builder",
        ":plan_status_lcm",
        ":bdisc_parser",
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
	"//drake/systems/analysis:runge_kutta3_integrator",
	"//drake/systems/analysis:semi_explicit_euler_integrator",
	"//drake/systems/framework:diagram_builder",
	"//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:piecewise_polynomial",
	"//drake/common:find_resource",
	"//drake/common:text_logging_gflags",
	"@com_github_gflags_gflags//:gflags",
	"@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "valkyrie_simulator",
    srcs = ["src/simulators/valkyrie/valkyrie_simulator.cc"],
    deps = [
	":bdisc_parser",
        ":robot_state_lcm",
	":robot_plan_interpolator",
        ":world_sim_tree_builder",
        ":plan_status_lcm",
	":valkyrie_world_diagram",
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
	"//drake/systems/analysis:runge_kutta3_integrator",
	"//drake/systems/analysis:semi_explicit_euler_integrator",
	"//drake/systems/framework:diagram_builder",
	"//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:piecewise_polynomial",
	"//drake/common:find_resource",
	"//drake/common:text_logging_gflags",
	"@com_github_gflags_gflags//:gflags",
	"@lcmtypes_robotlocomotion",
    ],
)


cpplint()
