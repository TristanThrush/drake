#include <gflags/gflags.h>
#include <fstream>
#include <typeinfo>
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/bhpn_drake_interface/src/utils/robot_state_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/utils/plan_status_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/utils/world_sim_tree_builder.h"
#include "drake/examples/bhpn_drake_interface/src/utils/bdisc_parser.h"
#include "drake/examples/bhpn_drake_interface/src/simulators/valkyrie/valkyrie_world_diagram.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {


void main(int argc, char* argv[]) {

  bdisc_parser::simulator_conf conf = bdisc_parser::parse(argv[1]);
  
  // Create the appropriate simulator and controller, given the robot.
  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> diagram_builder;
  
  // Add the valkyrie world plant and set it up to take commands from a controller and publish state information.
  auto valkyrie_world_diagram = diagram_builder.AddSystem<ValkyrieWorldDiagram>(conf, &lcm);

  // Create the simulation.
  std::unique_ptr<systems::Diagram<double>> diagram = diagram_builder.Build(); 
  systems::Simulator<double> simulator(*diagram);
  auto context = simulator.get_mutable_context();

  
  simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      *diagram, 2e-4, context);
  simulator.set_publish_every_time_step(false);
  
  /*
  simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
      *diagram, context);
  simulator.get_mutable_integrator()->set_target_accuracy(1e-3);
  simulator.get_mutable_integrator()->set_maximum_step_size(5e-1);
  */
  /*
  simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(*diagram, context);
  simulator.get_mutable_integrator()->set_target_accuracy(5e-1);
  simulator.get_mutable_integrator()->set_maximum_step_size(5e-1);
  */
  // Set the initial joint positions.
  VectorX<double> initial_robot_joint_positions_no_floating_joints(conf.initial_robot_joint_positions.size() - 6);
  auto plant_ = valkyrie_world_diagram->get_mutable_plant();
  for (int index = 0; index < (int)conf.initial_robot_joint_positions.size();
       index++) {
    plant_->set_position(simulator.get_mutable_context(), index,
                       conf.initial_robot_joint_positions[index]);
    if (index >= 6){
      initial_robot_joint_positions_no_floating_joints[index-6] = conf.initial_robot_joint_positions[index];
    }
  }
  std::cout << "init: " << initial_robot_joint_positions_no_floating_joints << "\n";
   auto& plan_source_context = diagram->GetMutableSubsystemContext(
      *valkyrie_world_diagram->get_command_injector(), simulator.get_mutable_context());
  valkyrie_world_diagram->get_command_injector()->Initialize(plan_source_context.get_time(),
                               initial_robot_joint_positions_no_floating_joints,
                               plan_source_context.get_mutable_state());
  
  // Start the simulation.
  lcm.StartReceiveThread();
  simulator.StepTo(999999999999);
}
}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  drake::examples::bhpn_drake_interface::main(argc, argv);
  return 0;
}
