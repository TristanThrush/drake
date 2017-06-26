#include "drake/examples/bhpn_drake_interface/lcm_utils/body_position_data_lcm.h"

#include "drake/common/drake_assert.h"
#include "drake/lcmt_body_position_data.hpp"

namespace drake {
    namespace examples {
        namespace bhpn_drake_interface {

            using systems::BasicVector;
            using systems::Context;
            using systems::DiscreteValues;
            using systems::State;
            using systems::SystemOutput;


            const double lcmStatusPeriod = 0.005;

            BodyPositionDataSender::BodyPositionDataSender(int num_bodies)
                    : num_bodies_(num_bodies) {
                this->DeclareInputPort(systems::kVectorValued, num_bodies_ * 2);
                this->DeclareInputPort(systems::kVectorValued, num_bodies_ * 2);
                this->DeclareAbstractOutputPort(&BodyPositionDataSender::MakeOutputStatus,
                                                &BodyPositionDataSender::OutputStatus);
            }

            lcmt_body_position_data BodyPositionDataSender::MakeOutputStatus() const {
                lcmt_body_position_data msg{};
                msg.num_bodies = num_bodies_;
                msg.body_names.resize(msg.num_bodies, );
                msg.body_positions.resize(msg.num_bodies, 6, 0);
                return msg;
            }

            void BodyPositionDataSender::OutputStatus(
                    const Context<double> &context, lcmt_body_position_data *output) const {
                lcmt_body_position_data &status = *output;

                status.timestamp = context.get_time() * 1e6;
                const systems::BasicVector<double> *state =
                        this->EvalVectorInput(context, 1);
                for (int i = 0; i < num_bodies_; ++i) {
                    status.joint_position[i] = state->GetAtIndex(i);
                }
            }


        }  // namespace bhpn_drake_interface
    }  // namespace examples
}  // namespace drake
