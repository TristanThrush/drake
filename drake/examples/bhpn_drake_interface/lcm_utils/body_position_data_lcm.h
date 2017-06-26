#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_body_position_data.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
    namespace examples {
        namespace bhpn_drake_interface {

            extern const double lcmStatusPeriod;

            class BodyPositionDataSender : public systems::LeafSystem<double> {
            public:
                DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyPositionDataSender)

                explicit BodyPositionDataSender(int num_bodies);

                const systems::InputPortDescriptor<double> &get_command_input_port() const {
                    return this->get_input_port(0);
                }

                const systems::InputPortDescriptor<double> &get_state_input_port() const {
                    return this->get_input_port(1);
                }

            private:
                // This is the method to use for the output port allocator.
                lcmt_body_position_data MakeOutputStatus() const;

                // This is the calculator method for the output port.
                void OutputStatus(const systems::Context<double> &context,
                                  lcmt_body_position_data *output) const;

                const int num_bodies_;
            };

        }  // namespace bhpn_drake_interface
    }  // namespace examples
}  // namespace drake
