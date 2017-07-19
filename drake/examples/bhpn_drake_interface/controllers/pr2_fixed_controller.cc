#include "drake/examples/bhpn_drake_interface/controllers/pr2_fixed_controller.h"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

systems::PidController<double>* add_pr2_fixed_controller(systems::DiagramBuilder<double>* diagram_builder, systems::RigidBodyPlant<double>* plant, int pr2_fixed_instance_id,  systems::lcm::LcmSubscriberSystem* plan_receiver, RobotPlanInterpolator* command_injector) {

  //Create the controller.
  int num_actuators = 24;
  VectorX<double> kp(num_actuators);
  kp << 2000, 2000, 5000, 800000, 1000, 1000, 4000, 4100, 2000, 2000, 300, 100, 100, 50, 50, 4000, 4100, 2000, 2000, 300, 100, 100, 50, 50;
  kp *= 0.5;
  VectorX<double> ki(num_actuators);
  ki << 0, 0, 15, 50000, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 15, 15, 15, 15, 15, 15, 15, 0, 0;
  ki *= 0.3;
  VectorX<double> kd(num_actuators);
  kd << 18200, 18200, 7500, 7, 7, 7, 75, 50, 7, 7, 2, 7, 1, 1, 1, 75, 50, 7, 7, 2, 7, 1, 1, 1;
  kd *= 0.1;
  auto Binv = plant->get_rigid_body_tree().B.block(0, 0, num_actuators, num_actuators).inverse();
  auto controller = diagram_builder->AddSystem<systems::PidController<double>>(std::make_unique<systems::PidController<double>>(
      Binv, MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki,
      kd));
  
  //Wire things up.
  diagram_builder->Connect(plant->model_instance_state_output_port(pr2_fixed_instance_id), command_injector->get_state_input_port());
  diagram_builder->Connect(plan_receiver->get_output_port(0), command_injector->get_plan_input_port());
  diagram_builder->Connect(command_injector->get_state_output_port(), controller->get_input_port_desired_state());
  diagram_builder->Connect(plant->model_instance_state_output_port(pr2_fixed_instance_id), controller->get_input_port_estimated_state());
  diagram_builder->Connect(controller->get_output_port_control(), plant->model_instance_actuator_command_input_port(pr2_fixed_instance_id));

  //Return the controller.
  return controller;
  
}
}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake
