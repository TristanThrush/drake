#include "drake/examples/bhpn_drake_interface/lcm_utils/piecewise_polynomial_lcm.h"

#include "drake/common/drake_assert.h"
#include "drake/lcmt_robot_state.hpp"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::State;
using systems::SystemOutput;

const double lcmStatusPeriod = 0.005;

PiecewisePolynomialReceiver::PiecewisePolynomialReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(num_joints_ * 2),
                                &PiecewisePolynomialReceiver::OutputCommand);
  this->DeclarePeriodicDiscreteUpdate(lcmStatusPeriod);
  this->DeclareDiscreteState(num_joints_ * 2);
}

void PiecewisePolynomialReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value =
      context->get_mutable_discrete_state(0)->get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.head(num_joints_) = x;
  state_value.tail(num_joints_) = VectorX<double>::Zero(num_joints_);
}

void PiecewisePolynomialReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& msg = input->GetValue<lcmt_piecewise_polynomial>();
  const auto& piecewise_polynomial = decodePiecewisePolynomial(msg);
  const auto& piecewise_polynomial_trajectory =
      PiecewisePolynomialTrajectory(piecewise_polynomial);
  const auto& trajectory_source =
      TrajectorySource<double>(*piecewise_polynomial_trajectory, 1);
  Eigen::VectorBlock<VectorX<double>>* output;
  trajectory_source.DoCalcVectorOutput(context, output);
  
  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (output.size() != 0) {
    DRAKE_DEMAND(output.size() == num_joints_);
    /*
    VectorX<double> new_positions(num_joints_);
    for (int i = 0; i < command.num_joints; ++i) {
      new_positions(i) = command.joint_position[i];
    }
    */
    BasicVector<double>* state = discrete_state->get_mutable_vector(0);
    auto state_value = state->get_mutable_value();
    state_value.tail(num_joints_) =
        (new_positions - state_value.head(num_joints_)) / lcmStatusPeriod;
    state_value.head(num_joints_) = output;
  }
}

void PiecewisePolynomialReceiver::OutputCommand(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0)->get_value();
}

}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake
