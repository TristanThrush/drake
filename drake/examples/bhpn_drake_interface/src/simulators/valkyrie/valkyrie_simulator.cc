#include <gflags/gflags.h>
#include "gperftools/profiler.h"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/bhpn_drake_interface/src/utils/robot_state_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/utils/plan_status_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/utils/world_sim_tree_builder.h"
#include "drake/examples/bhpn_drake_interface/src/simulators/valkyrie/valkyrie_controller_diagram.h"
#include "drake/examples/bhpn_drake_interface/src/simulators/valkyrie/valkyrie_world_diagram.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_robot_state.hpp"
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

  // Create the appropriate simulator and controller, given the robot.
  drake::lcm::DrakeLcm lcm;
   
  systems::DiagramBuilder<double> diagram_builder;
  
  // Add the valkyrie world plant and set it up to take commands from a controller and publish state information.
  diagram_builder.AddSystem<ValkyrieWorldDiagram>(&lcm);
  
  // Create the simulation.
  std::unique_ptr<systems::Diagram<double>> diagram = diagram_builder.Build(); 
  systems::Simulator<double> simulator(*diagram);
  auto context = simulator.get_mutable_context();
  simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      *diagram, 3e-4, context);
  simulator.set_publish_every_time_step(false);

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
