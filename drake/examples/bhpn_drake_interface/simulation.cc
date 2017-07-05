#include "drake/common/drake_path.h"
#include "drake/examples/bhpn_drake_interface/lcm_utils/oracular_state_estimator.h"
#include "drake/examples/bhpn_drake_interface/lcm_utils/robot_state_lcm.h"
#include "drake/examples/bhpn_drake_interface/simulation_utils/sim_diagram_builder.h"
#include "drake/examples/bhpn_drake_interface/simulation_utils/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_robot_state.hpp"
#include "drake/lcmt_piecewise_polynomial.hpp"
#include "drake/math/autodiff.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

std::unique_ptr<systems::PidController<double>> valkyrie_controller(
    systems::RigidBodyPlant<double>* plant) {
  int num_actuators = 30;
  VectorX<double> kp(num_actuators);
  kp << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1;
  kp *= 1000;
  VectorX<double> ki(num_actuators);
  ki << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0;
  VectorX<double> kd(num_actuators);
  kd << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1;
  kd *= 5;
  auto Binv = plant->get_rigid_body_tree()
                  .B.block(0, 0, num_actuators, num_actuators)
                  .inverse();
  return std::make_unique<systems::PidController<double>>(
      Binv, MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki,
      kd);
}

std::unique_ptr<systems::PidController<double>> pr2_controller(
    systems::RigidBodyPlant<double>* plant) {
  int num_actuators = 21;
  VectorX<double> kp(num_actuators);
  kp << 200000, 300, 300, 300, 300, 300, 300, 40, 40, 40, 40, 40, 300, 300, 300, 300, 40, 40, 40, 40, 40;
  kp *= 0.5;
  VectorX<double> ki(num_actuators);
  ki << 5, 5, 5, 5, 5, 5, 5, 0, 0, 0, 0, 0, 5, 5, 5, 5, 0, 0, 0, 0, 0;
  //ki *= 0;
  VectorX<double> kd(num_actuators);
  kd << 7, 7, 7, 7, 7, 7, 7, 0, 0, 0, 0, 0, 7, 7, 7, 7, 0, 0, 0, 0, 0;
  //kd *= 0;
  auto Binv = plant->get_rigid_body_tree()
                  .B.block(0, 0, num_actuators, num_actuators)
                  .inverse();
  return std::make_unique<systems::PidController<double>>(
      Binv, MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki,
      kd);
}

std::unique_ptr<systems::InverseDynamicsController<double>> iiwa_controller(
    std::unique_ptr<RigidBodyTree<double>> tree_) {
  int num_actuators = 7;
  VectorX<double> kp(num_actuators);
  kp << 100, 100, 100, 100, 100, 100, 100;
  VectorX<double> ki(num_actuators);
  ki << 0, 0, 0, 0, 0, 0, 0;
  VectorX<double> kd(num_actuators);
  kd << 2, 2, 2, 2, 2, 2, 2;
  return std::make_unique<systems::InverseDynamicsController<double>>(
      std::move(tree_), kp, ki, kd, false);
}

std::unique_ptr<RigidBodyTree<double>> build_world_tree(
    std::vector<ModelInstanceInfo<double>>* world_info,
    std::vector<std::string> urdf_paths, std::vector<Vector3d> poses_xyz,
    std::vector<Vector3d> poses_rpy, std::vector<std::string> fixed) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  for (auto urdf : urdf_paths) {
    tree_builder->StoreModel(urdf, urdf);
  }
  world_info->clear();
  for (int index = 0; index < (int)urdf_paths.size(); index++) {
    if (!(fixed[index]).compare("true")) {
      int object_id = tree_builder->AddFixedModelInstance(
          urdf_paths[index], poses_xyz[index], poses_rpy[index]);
      world_info->push_back(
          tree_builder->get_model_info_for_instance(object_id));
    } else {
      int object_id = tree_builder->AddFloatingModelInstance(
          urdf_paths[index], poses_xyz[index], poses_rpy[index]);
      world_info->push_back(
          tree_builder->get_model_info_for_instance(object_id));
    }
  }

  tree_builder->AddGround();
  return tree_builder->Build();
}

void main(int argc, char* argv[]) {
  // parse the arguments
  std::vector<std::string> urdf_paths;
  std::vector<Vector3d> poses_xyz;
  std::vector<Vector3d> poses_rpy;
  std::vector<std::string> fixed;
  std::vector<double> initial_joint_positions;
  int num_actuators = std::stoi(argv[1]);
  std::string robot_name(argv[num_actuators + 2]);

  for(int index = 2; index < num_actuators + 2; index ++){
	initial_joint_positions.push_back(std::stod(argv[index]));
  }

  for (int index = num_actuators + 3; index < argc; index++) {
    urdf_paths.push_back(argv[index]);
    index++;
    double x = std::stod(argv[index]);
    index++;
    double y = std::stod(argv[index]);
    index++;
    double z = std::stod(argv[index]);
    poses_xyz.push_back(Vector3d(x, y, z));
    index++;
    double roll = std::stod(argv[index]);
    index++;
    double pitch = std::stod(argv[index]);
    index++;
    double yaw = std::stod(argv[index]);
    poses_rpy.push_back(Vector3d(roll, pitch, yaw));
    index++;
    fixed.push_back(argv[index]);
  }

  // construct the world and initialize some other things
  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> world_info;
  SimDiagramBuilder<double> builder;
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();
  systems::RigidBodyPlant<double>* plant = builder.AddPlant(
      build_world_tree(&world_info, urdf_paths, poses_xyz, poses_rpy, fixed));
  //auto num_actuators = plant->get_rigid_body_tree().get_num_actuators();
  std::cout << "num_actuators: " << num_actuators << "\n";
  std::cout << "actuators: " << "\n";
  for (RigidBodyActuator actuator : plant->get_rigid_body_tree().actuators){
	std::cout << actuator.name_ << "\n";
  }
  builder.AddVisualizer(&lcm);
  
  // set up communication with BHPN
  auto command_sub = diagram_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_robot_state>(
          "BHPN_ROBOT_STATE_COMMAND", &lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      diagram_builder->AddSystem<RobotStateReceiver>(num_actuators);
  command_receiver->set_name("command_receiver");
  
  /*
  auto command_sub = diagram_builder->AddSystem(
	systems::lcm::LcmSubscriberSystem::Make<lcmt_piecewise_polynomial>(
		"TRAJECTORY_COMMAND", &lcm));
	command_sub->set_name("command_subscriber");
  auto piecewise_polynomial_value = command_sub->get_output_port(0).Calc(*command_sub->CreateDefaultContext().get(), &command_sub->get_output_port(0).Allocate(*command_sub->CreateDefaultContext().get())).GetValue<lcmt_piecewise_polynomial>();
  auto command_receiver = 
	diagram_builder->AddSystem<systems::TrajectorySource<double>>(PiecewisePolynomialTrajectory(decodePiecewisePolynomial(piecewise_polynomial_value)), 1);
	command_receiver->set_name("command_reciever");
*/
  auto status_pub = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_robot_state>(
          "DRAKE_ROBOT_STATE", &lcm));

  status_pub->set_name("status_publisher");
  status_pub->set_publish_period(lcmStatusPeriod);
  auto status_sender =
      diagram_builder->AddSystem<RobotStateSender>(num_actuators);
  status_sender->set_name("status_sender");

  diagram_builder->Connect(command_sub->get_output_port(0),
                           command_receiver->get_input_port(0));

  // connect the appropriate controller for the type of robot
  const auto& state_out_port =
      plant->model_instance_state_output_port(world_info[0].instance_id);
  auto demux = diagram_builder->template AddSystem<systems::Demultiplexer>(
      state_out_port.size());
  auto mux = diagram_builder->template AddSystem<systems::Multiplexer>(
      num_actuators * 2);
  diagram_builder->Connect(state_out_port, demux->get_input_port(0));

  for (int index = 0; index < num_actuators * 2; index++) {
    diagram_builder->Connect(demux->get_output_port(index),
                             mux->get_input_port(index));
  }

  if (!robot_name.compare("pr2")) {
    auto controller =
        diagram_builder->AddSystem<systems::PidController<double>>(
            pr2_controller(plant));
    diagram_builder->Connect(command_receiver->get_output_port(0),
                             controller->get_input_port_desired_state());
    diagram_builder->Connect(mux->get_output_port(0),
                             controller->get_input_port_estimated_state());
    diagram_builder->Connect(controller->get_output_port_control(),
                             plant->model_instance_actuator_command_input_port(
                                 world_info[0].instance_id));
  } else if (!robot_name.compare("iiwa")) {
    auto single_arm = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        world_info[0].model_path, multibody::joints::kFixed,
        world_info[0].world_offset, single_arm.get());
    auto controller =
        diagram_builder->AddSystem<systems::InverseDynamicsController<double>>(
            iiwa_controller(std::move(single_arm)));
    diagram_builder->Connect(command_receiver->get_output_port(0),
                             controller->get_input_port_desired_state());
    diagram_builder->Connect(mux->get_output_port(0),
                             controller->get_input_port_estimated_state());
    diagram_builder->Connect(controller->get_output_port_control(),
                             plant->model_instance_actuator_command_input_port(
                                 world_info[0].instance_id));
  } else if (!robot_name.compare("valkyrie")) {
    auto controller =
        diagram_builder->AddSystem<systems::PidController<double>>(
            valkyrie_controller(plant));
    diagram_builder->Connect(command_receiver->get_output_port(0),
                             controller->get_input_port_desired_state());
    diagram_builder->Connect(mux->get_output_port(0),
                             controller->get_input_port_estimated_state());
    diagram_builder->Connect(controller->get_output_port_control(),
                             plant->model_instance_actuator_command_input_port(
                                 world_info[0].instance_id));
  }

  diagram_builder->Connect(
      mux->get_output_port(
          0),  // plant->model_instance_state_output_port(world_info[0].instance_id),
      status_sender->get_state_input_port());
  diagram_builder->Connect(command_receiver->get_output_port(0),
                           status_sender->get_command_input_port());
  diagram_builder->Connect(status_sender->get_output_port(0),
                           status_pub->get_input_port(0));
  
  // start the simulation
  lcm.StartReceiveThread();
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  for(int index = 0; index < num_actuators; index++){
     simulator.get_mutable_context()->get_mutable_continuous_state_vector()->SetAtIndex(index, initial_joint_positions[index]);
  }
  std::cout << simulator.get_mutable_context()->get_mutable_continuous_state_vector()->CopyToVector() << "\n";
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(5000000000);
}
}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  drake::examples::bhpn_drake_interface::main(argc, argv);
  return 0;
}
