#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_joint_positions.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pr2 {

class JointPositionsSender : public systems::LeafSystem<double> {
 public:
   DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointPositionsSender)

   explicit JointPositionsSender(int num_joints);

 private:
   // This is the method to use for the output port allocator.
   lcmt_joint_positions MakeOutputStatus() const;

   // This is the calculator method for the output port.
   void OutputStatus(const systems::Context<double>& context,
                    lcmt_joint_positions* output) const;

   const int num_joints_;
};

}  // namespace pr2
}  // namespace examples
}  // namespace drake
