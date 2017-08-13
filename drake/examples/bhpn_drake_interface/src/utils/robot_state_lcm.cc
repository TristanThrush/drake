#include "drake/examples/bhpn_drake_interface/src/utils/robot_state_lcm.h"

#include "drake/common/drake_assert.h"
#include "lcmtypes/bot_core/robot_state_t.hpp"

namespace drake {
namespace examples {
namespace bhpn_drake_interface {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::State;
using systems::SystemOutput;

const double lcmStatusPeriod = 0.005;

RobotStateReceiver::RobotStateReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(num_joints_ * 2),
                                &RobotStateReceiver::OutputCommand);
  this->DeclarePeriodicDiscreteUpdate(lcmStatusPeriod);
  this->DeclareDiscreteState(num_joints_ * 2);
}

void RobotStateReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value =
      context->get_mutable_discrete_state(0)->get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.head(num_joints_) = x;
  state_value.tail(num_joints_) = VectorX<double>::Zero(num_joints_);
}

void RobotStateReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<bot_core::robot_state_t>();

  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (command.num_joints != 0) {
    DRAKE_DEMAND(command.num_joints == num_joints_);
    VectorX<double> new_positions(num_joints_);
    for (int i = 0; i < command.num_joints; ++i) {
      new_positions(i) = command.joint_position[i];
    }

    BasicVector<double>* state = discrete_state->get_mutable_vector(0);
    auto state_value = state->get_mutable_value();
    state_value.tail(num_joints_) =
        (new_positions - state_value.head(num_joints_)) / lcmStatusPeriod;
    state_value.head(num_joints_) = new_positions;
  }
}

void RobotStateReceiver::OutputCommand(const Context<double>& context,
                                       BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0)->get_value();
}

RobotStateSender::RobotStateSender(int num_joints) : num_joints_(num_joints) {
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  this->DeclareAbstractOutputPort(&RobotStateSender::MakeOutputStatus,
                                  &RobotStateSender::OutputStatus);
}

bot_core::robot_state_t RobotStateSender::MakeOutputStatus() const {
  bot_core::robot_state_t msg{};
  msg.num_joints = num_joints_;
  msg.joint_position.resize(msg.num_joints, 0);
  msg.joint_name.resize(msg.num_joints, "");
  msg.joint_velocity.resize(msg.num_joints, 0);
  msg.joint_effort.resize(msg.num_joints, 0);
  return msg;
}

void RobotStateSender::OutputStatus(const Context<double>& context,
                                    bot_core::robot_state_t* output) const {
  bot_core::robot_state_t& status = *output;

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* state = this->EvalVectorInput(context, 1);
  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position[i] = state->GetAtIndex(i);
  }
}

}  // namespace bhpn_drake_interface
}  // namespace examples
}  // namespace drake
