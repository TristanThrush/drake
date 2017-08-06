#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/examples/valkyrie/actuator_effort_to_rigid_body_plant_input_converter.h"
#include "drake/examples/valkyrie/robot_command_to_desired_effort_converter.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

std::unique_ptr<RigidBodyTree<double>> build_world_tree(
    std::vector<ModelInstanceInfo<double>>* world_info,
    std::vector<std::string> names, std::vector<std::string> description_paths,
    std::vector<Eigen::Vector3d> poses_xyz,
    std::vector<Eigen::Vector3d> poses_rpy, std::vector<bool> fixed) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  for (int index = 0; index < (int)names.size(); index++) {
    tree_builder->StoreModel(names[index], description_paths[index]);
  }
  world_info->clear();
  for (int index = 0; index < (int)names.size(); index++) {
    if (fixed[index]) {
      int object_id = tree_builder->AddFixedModelInstance(
          names[index], poses_xyz[index], poses_rpy[index]);
      world_info->push_back(
          tree_builder->get_model_info_for_instance(object_id));
    } else {
      int object_id = tree_builder->AddFloatingModelInstance(
          names[index], poses_xyz[index], poses_rpy[index]);
      world_info->push_back(
          tree_builder->get_model_info_for_instance(object_id));
    }
  }

  tree_builder->AddGround();
  return tree_builder->Build();
}

class ValkyrieWorldDiagram : public systems::Diagram<double> {
 public:
  explicit ValkyrieWorldDiagram(bdisc_parser::simulator_conf conf, drake::lcm::DrakeLcm* lcm) {
    systems::DiagramBuilder<double> builder;

    // Create RigidBodyTree with just Valkyrie
    valkyrie_tree_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        drake::FindResourceOrThrow("drake/examples/Valkyrie/urdf/urdf/valkyrie.urdf"), multibody::joints::kRollPitchYaw, valkyrie_tree_.get());

    // Create RigidBodyTree with Valkyrie and other objects and use this one to construct the plant.
    std::vector<ModelInstanceInfo<double>> world_info;
    std::vector<std::string> names = conf.object_names;
    names.insert(names.begin(), "valkyrie");
    std::vector<std::string> description_paths = conf.object_description_paths;
    description_paths.insert(description_paths.begin(), "drake/examples/Valkyrie/urdf/urdf/valkyrie.urdf");
    std::vector<Eigen::Vector3d> initial_poses_xyz = conf.initial_object_poses_xyz;
    initial_poses_xyz.insert(initial_poses_xyz.begin(), conf.initial_robot_pose_xyz);
    std::vector<Eigen::Vector3d> initial_poses_rpy = conf.initial_object_poses_rpy;
    initial_poses_rpy.insert(initial_poses_rpy.begin(), conf.initial_robot_pose_rpy);
    std::vector<bool> fixed = conf.object_fixed;
    fixed.insert(fixed.begin(), false);

    plant_ =
        builder.AddSystem<systems::RigidBodyPlant<double>>(build_world_tree(&world_info, names, description_paths, initial_poses_xyz, initial_poses_rpy, fixed));
    plant_->set_name("plant");

    // Set up a state estimator that generates robot_state_t messages.    
    auto valkyrie_state_est_ = builder.template AddSystem<kuka_iiwa_arm::OracularStateEstimation<double>>(*valkyrie_tree_);
    valkyrie_state_est_->set_name("OracularStateEstimationValkyrieState");
    builder.Connect(plant_->model_instance_state_output_port(world_info[0].instance_id), valkyrie_state_est_->get_input_port_state());
    auto& robot_state_publisher = *builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
            "EST_ROBOT_STATE", lcm));
    robot_state_publisher.set_name("robot_state_publisher");
    robot_state_publisher.set_publish_period(1e-3);
    builder.Connect(valkyrie_state_est_->get_output_port_msg(), robot_state_publisher.get_input_port(0));	

    // Contact parameters
    const double kStiffness = 1000;
    const double kDissipation = 100.0;
    const double kStaticFriction = 1.0;
    const double kDynamicFriction = 0.5;
    const double kStictionSlipTolerance = 0.4;
    plant_->set_normal_contact_parameters(kStiffness, kDissipation);
    plant_->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                            kStictionSlipTolerance);
    const auto& tree = plant_->get_rigid_body_tree();


    // RigidBodyActuators.
    std::vector<const RigidBodyActuator*> actuators;
    for (const auto& actuator : tree.actuators) {
      std::cout << "actuator name: " << actuator.name_ << "\n";
      actuators.push_back(&actuator);
    }
    // Currently, all RigidBodyActuators are assumed to be one-dimensional.
    const int actuator_effort_length = 1;

    // LCM inputs.
    auto& atlas_command_subscriber = *builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<bot_core::atlas_command_t>(
            "ROBOT_COMMAND", lcm));
    atlas_command_subscriber.set_name("atlas_command_subscriber");

    auto& robot_command_to_desired_effort_converter =
        *builder.AddSystem<systems::RobotCommandToDesiredEffortConverter>(
            actuators);
    robot_command_to_desired_effort_converter.set_name(
        "robot_command_to_desired_effort_converter");

    // Placeholder for actuator dynamics.
    std::map<const RigidBodyActuator*, System<double>*> actuator_dynamics;
    for (const auto& actuator : actuators) {
      auto pass_through_ptr = builder.AddSystem<systems::PassThrough<double>>(
          actuator_effort_length);
      pass_through_ptr->set_name(actuator->name_ + "_actuator_dynamics");

      actuator_dynamics.emplace(std::make_pair(actuator, pass_through_ptr));
    }

    // Conversion from desired efforts to RigidBodyPlant input vector.
    auto& actuator_effort_to_rigid_body_plant_input_converter =
        *builder
             .AddSystem<systems::ActuatorEffortToRigidBodyPlantInputConverter>(
                 actuators);
    actuator_effort_to_rigid_body_plant_input_converter.set_name(
        "actuator_effort_to_rigid_body_plant_input_converter");

    // Placeholder for effort sensors.
    std::map<const RigidBodyActuator*, System<double>*> effort_sensors;
    for (const auto& actuator : actuators) {
      auto pass_through_ptr = builder.AddSystem<systems::PassThrough<double>>(
          actuator_effort_length);
      pass_through_ptr->set_name(actuator->name_ + "_trq_sensor");

      effort_sensors.emplace(std::make_pair(actuator, pass_through_ptr));
    }

    // Visualizer.
    systems::DrakeVisualizer& visualizer_publisher =
        *builder.template AddSystem<systems::DrakeVisualizer>(tree, lcm);
    visualizer_publisher.set_name("visualizer_publisher");

    systems::ContactResultsToLcmSystem<double>& contact_viz =
        *builder.template AddSystem<systems::ContactResultsToLcmSystem<double>>(
            tree);
    contact_viz.set_name("contact_viz");

    auto& contact_results_publisher = *builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", lcm));
    contact_results_publisher.set_name("contact_results_publisher");

    contact_results_publisher.set_publish_period(1e-3);
    visualizer_publisher.set_publish_period(1e-3);
  

    // Connections.
    // LCM message to desired effort conversion.
    builder.Connect(atlas_command_subscriber,
                    robot_command_to_desired_effort_converter);

    for (const auto& actuator : actuators) {

      // Desired effort inputs to actuator dynamics.
          const auto& desired_effort_output =
          robot_command_to_desired_effort_converter.desired_effort_output_port(
              *actuator);
      const auto& desired_effort_input =
          actuator_dynamics.at(actuator)->get_input_port(0);
      builder.Connect(desired_effort_output, desired_effort_input);

      // Efforts to effort sensors.
      const auto& effort_output_port =
          actuator_dynamics.at(actuator)->get_output_port(0);
      const auto& measured_effort_input_port =
          effort_sensors.at(actuator)->get_input_port(0);
      builder.Connect(effort_output_port, measured_effort_input_port);

      // Efforts to rigid body plant input
      builder.Connect(
          effort_output_port,
          actuator_effort_to_rigid_body_plant_input_converter.effort_input_port(
              *actuator));     
    }

    // Plant input to plant.
    builder.Connect(
        actuator_effort_to_rigid_body_plant_input_converter.get_output_port(0),
        plant_->get_input_port(0));

    // Raw state vector to visualizer.
    builder.Connect(plant_->state_output_port(),
                    visualizer_publisher.get_input_port(0));

    // Contact results to lcm msg.
    builder.Connect(plant_->contact_results_output_port(),
                    contact_viz.get_input_port(0));
    builder.Connect(contact_viz.get_output_port(0),
                    contact_results_publisher.get_input_port(0));

    builder.BuildInto(this);
  }

  systems::RigidBodyPlant<double>* get_mutable_plant() { return plant_; }

 private:
  systems::RigidBodyPlant<double>* plant_;
  std::unique_ptr<RigidBodyTree<double>> valkyrie_tree_;
};

}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake
