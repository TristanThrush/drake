#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/common/find_resource.h"
#include "drake/examples/bhpn_drake_interface/src/lcm_utils/robot_plan_interpolator.h"
#include "drake/examples/bhpn_drake_interface/src/lcm_utils/robot_state_lcm.h"
#include "drake/examples/bhpn_drake_interface/src/simulation_utils/sim_diagram_builder.h"
#include "drake/examples/bhpn_drake_interface/src/simulation_utils/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_robot_state.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

systems::PidController<double>* add_pr2_fixed_controller(
    systems::DiagramBuilder<double>* diagram_builder,
    systems::RigidBodyPlant<double>* plant, int pr2_fixed_instance_id,
    systems::lcm::LcmSubscriberSystem* plan_receiver,
    RobotPlanInterpolator* command_injector);

}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake
