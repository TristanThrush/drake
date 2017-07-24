#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/bhpn_drake_interface/src/controllers/pr2_fixed_controller.h"
#include "drake/examples/bhpn_drake_interface/src/lcm_utils/robot_state_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/lcm_utils/plan_status_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/simulation_utils/sim_diagram_builder.h"
#include "drake/examples/bhpn_drake_interface/src/simulation_utils/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_robot_state.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

std::unique_ptr<RigidBodyTree<double>> build_world_tree(
    std::vector<ModelInstanceInfo<double>>* world_info,
    std::vector<std::string> names, std::vector<std::string> urdf_paths,
    std::vector<Eigen::Vector3d> poses_xyz,
    std::vector<Eigen::Vector3d> poses_rpy, std::vector<std::string> fixed) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  for (int index = 0; index < (int)names.size(); index++) {
    tree_builder->StoreModel(names[index], urdf_paths[index]);
  }
  world_info->clear();
  for (int index = 0; index < (int)names.size(); index++) {
    if (!(fixed[index]).compare("true")) {
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

void main(int argc, char* argv[]) {
  // Parse the arguments.
  std::vector<std::string> urdf_paths;
  std::vector<std::string> names;
  std::vector<Eigen::Vector3d> poses_xyz;
  std::vector<Eigen::Vector3d> poses_rpy;
  std::vector<std::string> fixed;
  int num_actuators = std::stoi(argv[1]);
  Eigen::VectorXd initial_joint_positions(num_actuators);

  for (int index = 2; index < num_actuators + 2; index++) {
    initial_joint_positions[index - 2] = std::stod(argv[index]);
  }

  for (int index = num_actuators + 2; index < argc; index++) {
    names.push_back(argv[index]);
    index++;
    urdf_paths.push_back(argv[index]);
    index++;
    double x = std::stod(argv[index]);
    index++;
    double y = std::stod(argv[index]);
    index++;
    double z = std::stod(argv[index]);
    poses_xyz.push_back(Eigen::Vector3d(x, y, z));
    index++;
    double roll = std::stod(argv[index]);
    index++;
    double pitch = std::stod(argv[index]);
    index++;
    double yaw = std::stod(argv[index]);
    poses_rpy.push_back(Eigen::Vector3d(roll, pitch, yaw));
    index++;
    fixed.push_back(argv[index]);
  }

  // Construct the world and initialize some other things.
  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> world_info;
  SimDiagramBuilder<double> builder;
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();
  systems::RigidBodyPlant<double>* plant = builder.AddPlant(build_world_tree(
      &world_info, names, urdf_paths, poses_xyz, poses_rpy, fixed));
  std::cout << "num_actuators: " << num_actuators << "\n";
  std::cout << "actuators: "
            << "\n";
  for (RigidBodyActuator actuator : plant->get_rigid_body_tree().actuators) {
    std::cout << actuator.name_ << "\n";
  }
  builder.AddVisualizer(&lcm);

  // Set up communication with BHPN.
  auto status_pub = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_robot_state>("ROBOT_STATE",
                                                               &lcm));
  status_pub->set_name("status_publisher");
  status_pub->set_publish_period(lcmStatusPeriod);

  auto status_sender =
      diagram_builder->AddSystem<RobotStateSender>(num_actuators);
  status_sender->set_name("status_sender");

  auto plan_receiver = diagram_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<robotlocomotion::robot_plan_t>(
          "ROBOT_PLAN", &lcm));
  plan_receiver->set_name("plan_receiver");

  auto command_injector = diagram_builder->AddSystem<RobotPlanInterpolator>(
      FindResourceOrThrow(urdf_paths[0]));
  command_injector->set_name("command_injector");
  
  auto plan_status_pub = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::plan_status_t>("PLAN_STATUS",
                                                               &lcm));
  plan_status_pub->set_name("plan_status_publisher");
  plan_status_pub->set_publish_period(lcmStatusPeriod);

  auto plan_status_sender = diagram_builder->AddSystem<PlanStatusSender>();
  plan_status_sender->set_name("plan_status_sender");

  add_pr2_fixed_controller(diagram_builder, plant, world_info[0].instance_id,
                           plan_receiver, command_injector);

  diagram_builder->Connect(command_injector->get_status_output_port(), plan_status_sender->get_input_port(0));
  diagram_builder->Connect(plan_status_sender->get_output_port(0), plan_status_pub->get_input_port(0));

  diagram_builder->Connect(
      plant->model_instance_state_output_port(world_info[0].instance_id),
      status_sender->get_state_input_port());
  diagram_builder->Connect(command_injector->get_state_output_port(),
                           status_sender->get_command_input_port());
  diagram_builder->Connect(status_sender->get_output_port(0),
                           status_pub->get_input_port(0));

  // Publish the contact results.
  auto contact_viz =
      diagram_builder->AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant->get_rigid_body_tree());
  auto contact_results_publisher = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));

  diagram_builder->Connect(plant->contact_results_output_port(),
                           contact_viz->get_input_port(0));
  diagram_builder->Connect(contact_viz->get_output_port(0),
                           contact_results_publisher->get_input_port(0));

  // Give the plant some contact parameters that encourage the gripping of
  // objects.

  const double kStiffness = 3000;
  const double kDissipation = 10.0;

  const double kStaticFriction = 1.8;
  const double kDynamicFriction = 1.0;
  const double kVStictionTolerance = 1e-5;
  plant->set_normal_contact_parameters(kStiffness, kDissipation);
  plant->set_friction_contact_parameters(kStaticFriction, kDynamicFriction, kVStictionTolerance);

  // Initialize the starting configuration of the joints, initialize the
  // command_injector, and start the simulation.
  lcm.StartReceiveThread();
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  const double dt = 1e-3;
  // simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(*diagram,
  // simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(dt);
  // simulator.get_mutable_integrator()->set_target_accuracy(0.5);

  for (int index = 0; index < plant->get_rigid_body_tree().get_num_actuators();
       index++) {
    plant->set_position(simulator.get_mutable_context(), index,
                        initial_joint_positions[index]);
  }
  auto& plan_source_context = diagram->GetMutableSubsystemContext(
      *command_injector, simulator.get_mutable_context());
  command_injector->Initialize(plan_source_context.get_time(),
                               initial_joint_positions,
                               plan_source_context.get_mutable_state());
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(500000000000);
}
}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  // gflags::ParseCommandLineFlags(&argc, &argv, true);
  // drake::logging::HandleSpdlogGflags();
  drake::examples::bhpn_drake_interface::main(argc, argv);
  return 0;
}
