#include "drake/examples/pr2/src/utils/joint_positions_lcm.h"

#include "drake/lcmt_joint_positions.hpp"

namespace drake {
namespace examples {
namespace pr2 {

JointPositionsSender::JointPositionsSender(int num_joints) : num_joints_(num_joints) {
  this->DeclareInputPort(systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(&JointPositionsSender::MakeOutputStatus,
                                  &JointPositionsSender::OutputStatus);
}

lcmt_joint_positions JointPositionsSender::MakeOutputStatus() const {
  lcmt_joint_positions msg{};
  msg.num_joints = num_joints_;
  msg.joint_positions.resize(msg.num_joints, 0);
  return msg;
}

void JointPositionsSender::OutputStatus(const systems::Context<double>& context,
                                    lcmt_joint_positions* output) const {
  lcmt_joint_positions& msg = *output;

  msg.timestamp = context.get_time() * 1e6;
  const systems::BasicVector<double>* positions = this->EvalVectorInput(context, 0);
  for (int i = 0; i < num_joints_; ++i) {
    msg.joint_positions[i] = positions->GetAtIndex(i);
  }
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake
