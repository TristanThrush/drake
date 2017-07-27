#include <gflags/gflags.h>
#include "gperftools/profiler.h"
#include <yaml-cpp/yaml.h>
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/bhpn_drake_interface/src/utils/robot_state_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/utils/plan_status_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/utils/world_sim_tree_builder.h"
#include "drake/examples/bhpn_drake_interface/src/utils/robot_plan_interpolator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_robot_state.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"


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

void main(int argc, char* argv[]) {

  std::cout << "yoooo" << "\n";

  // Declare diagram builder and lcm
  systems::DiagramBuilder<double> diagram_builder;
  drake::lcm::DrakeLcm lcm;
  const int num_actuators = 24;

  // Parse the conf
  std::vector<ModelInstanceInfo<double>> world_info;
  std::vector<std::string> names;
  names.push_back("pr2");
  std::vector<std::string> description_paths;
  description_paths.push_back("drake/examples/PR2/pr2_with_joints_for_base_movement_and_limited_gripper_movement.urdf");
  std::vector<Eigen::Vector3d> initial_poses_xyz;
  Eigen::Vector3d initial_pr2_pose_xyz;
  initial_pr2_pose_xyz << 0, 0, 0;
  initial_poses_xyz.push_back(initial_pr2_pose_xyz);
  std::vector<Eigen::Vector3d> initial_poses_rpy;
  Eigen::Vector3d initial_pr2_pose_rpy;
  initial_pr2_pose_rpy << 0, 0, 0;
  initial_poses_rpy.push_back(initial_pr2_pose_rpy);
  std::vector<bool> fixed;
  fixed.push_back(true);
  std::cout << "toooo" << "\n";
  std::cout << argv[1] << "\n";
  YAML::Node conf = YAML::Load(""); //simulator_type: pr2_simulator");
  std::cout << "woooo" << "\n";
  auto simulator_type = conf["simulator_type"].as<std::string>();
  auto robot_state_t_topic = conf["robot_state_t_topic"].as<std::string>();
  auto robot_plan_t_topic = conf["robot_plan_t_topic"].as<std::string>();
  auto plan_status_t_topic = conf["plan_status_t_topic"].as<std::string>();
  auto contact_results_for_viz_topic = conf["contact_results_for_viz_topic"].as<std::string>();
  auto jp = conf["initial_joint_positions"];
  Eigen::VectorXd initial_joint_positions(24);
  initial_joint_positions << jp[0].as<double>(), jp[1].as<double>(), jp[2].as<double>(), jp[3].as<double>(), jp[4].as<double>(), jp[5].as<double>(), jp[6].as<double>(), jp[7].as<double>(), jp[8].as<double>(), jp[9].as<double>(), jp[10].as<double>(), jp[11].as<double>(), jp[12].as<double>(), jp[13].as<double>(), jp[14].as<double>(), jp[15].as<double>(), jp[16].as<double>(), jp[17].as<double>(), jp[18].as<double>(), jp[19].as<double>(), jp[20].as<double>(), jp[21].as<double>(), jp[22].as<double>(), jp[23].as<double>();
 
  for (auto object : conf["objects"]){
	names.push_back(object["name"].as<std::string>());
	description_paths.push_back(object["description_path"].as<std::string>());
	auto initial_pose = object["initial_pose"];
	Eigen::Vector3d initial_pose_xyz;
	initial_pose_xyz << initial_pose[0].as<double>(), initial_pose[1].as<double>(), initial_pose[2].as<double>();
        initial_poses_xyz.push_back(initial_pose_xyz);
	Eigen::Vector3d initial_pose_rpy;
	initial_pose_xyz << initial_pose[3].as<double>(), initial_pose[4].as<double>(), initial_pose[5].as<double>();
        initial_poses_rpy.push_back(initial_pose_rpy);
	fixed.push_back(object["fixed"].as<bool>());
   }
  std::cout << "boooo" << "\n";
  // Construct the world tree, throw it into a plant, and add it to our diagram.
  auto plant_ = diagram_builder.AddSystem<systems::RigidBodyPlant<double>>(build_world_tree(&world_info, names, description_paths, initial_poses_xyz, initial_poses_rpy, fixed));
  plant_->set_name("plant_");
  auto pr2_instance_id = world_info[0].instance_id;

  // Set up communication with BHPN.
  auto robot_state_pub = diagram_builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_robot_state>(robot_state_t_topic,
                                                               &lcm));
  robot_state_pub->set_name("robot_state_publisher");
  robot_state_pub->set_publish_period(lcmStatusPeriod);

  auto robot_state_sender =
      diagram_builder.AddSystem<RobotStateSender>(num_actuators);
  robot_state_sender->set_name("robot_state_sender");

  auto plan_receiver = diagram_builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<robotlocomotion::robot_plan_t>(
          robot_plan_t_topic, &lcm));
  plan_receiver->set_name("plan_receiver");

  auto command_injector = diagram_builder.AddSystem<RobotPlanInterpolator>(
      FindResourceOrThrow(description_paths[0]));
  command_injector->set_name("command_injector");
  
  auto plan_status_pub = diagram_builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::plan_status_t>(plan_status_t_topic,
                                                               &lcm));
  plan_status_pub->set_name("plan_status_publisher");
  plan_status_pub->set_publish_period(lcmStatusPeriod);

  auto plan_status_sender = diagram_builder.AddSystem<PlanStatusSender>();
  plan_status_sender->set_name("plan_status_sender");

  diagram_builder.Connect(command_injector->get_status_output_port(), plan_status_sender->get_input_port(0));
  diagram_builder.Connect(plan_status_sender->get_output_port(0), plan_status_pub->get_input_port(0));

  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      robot_state_sender->get_state_input_port());
  diagram_builder.Connect(command_injector->get_state_output_port(),
                           robot_state_sender->get_command_input_port());
  diagram_builder.Connect(robot_state_sender->get_output_port(0),
                           robot_state_pub->get_input_port(0));

  // Add the controller and wire the appropriate systems together.
  VectorX<double> kp(num_actuators);
  kp << 2000, 2000, 5000, 800000, 1000, 1000, 4000, 4100, 2000, 2000, 400, 400,
      100, 50, 50, 4000, 4100, 2000, 2000, 400, 400, 100, 50, 50;
  kp *= 0.5;
  VectorX<double> ki(num_actuators);
  ki << 0, 0, 15, 50000, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 15, 15, 15,
      15, 15, 15, 15, 0, 0;
  ki *= 0.3;
  VectorX<double> kd(num_actuators);
  kd << 18200, 18200, 7500, 7, 7, 7, 75, 50, 7, 7, 2, 7, 1, 1, 1, 75, 50, 7, 7,
      2, 7, 1, 1, 1;
  kd *= 0.1;
  auto Binv = plant_->get_rigid_body_tree()
                  .B.block(0, 0, num_actuators, num_actuators)
                  .inverse();
  auto controller = diagram_builder.AddSystem<systems::PidController<double>>(
      std::make_unique<systems::PidController<double>>(
          Binv, MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki,
          kd));

  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      command_injector->get_state_input_port());
  diagram_builder.Connect(plan_receiver->get_output_port(0),
                           command_injector->get_plan_input_port());
  diagram_builder.Connect(command_injector->get_state_output_port(),
                           controller->get_input_port_desired_state());
  diagram_builder.Connect(
      plant_->model_instance_state_output_port(pr2_instance_id),
      controller->get_input_port_estimated_state());
  diagram_builder.Connect(
      controller->get_output_port_control(),
      plant_->model_instance_actuator_command_input_port(pr2_instance_id));

  // Add visualizer.
  auto visualizer_publisher = diagram_builder.template AddSystem<systems::DrakeVisualizer>(plant_->get_rigid_body_tree(), &lcm);
  visualizer_publisher->set_name("visualizer_publisher");
  visualizer_publisher->set_publish_period(lcmStatusPeriod);

  // Publish contact results.
  auto contact_viz =
      diagram_builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant_->get_rigid_body_tree());
  auto contact_results_publisher = diagram_builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          contact_results_for_viz_topic, &lcm));

  diagram_builder.Connect(plant_->contact_results_output_port(),
                           contact_viz->get_input_port(0));
  diagram_builder.Connect(contact_viz->get_output_port(0),
                           contact_results_publisher->get_input_port(0));

  // Create the simulation object.
  std::unique_ptr<systems::Diagram<double>> diagram = diagram_builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Set the initial joint positions.
  for (int index = 0; index < plant_->get_rigid_body_tree().get_num_actuators();
       index++) {
    plant_->set_position(simulator.get_mutable_context(), index,
                        initial_joint_positions[index]);
  }

  // Initialize the robot plan interpolator.
  auto& plan_source_context = diagram->GetMutableSubsystemContext(
      *command_injector, simulator.get_mutable_context());
  command_injector->Initialize(plan_source_context.get_time(),
                               initial_joint_positions,
                               plan_source_context.get_mutable_state());
  // Start the simulation.
  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(999999999999);
}
}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  drake::examples::bhpn_drake_interface::main(argc, argv);
  return 0;
}
