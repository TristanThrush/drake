#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_piecewise_polynomial.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
    namespace examples {
        namespace bhpn_drake_interface {

            extern const double lcmStatusPeriod;

            class PiecewisePolynomialReceiver : public systems::LeafSystem<double> {
            public:
                DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewisePolynomialReceiver)

                explicit PiecewisePolynomialReceiver(num_joints);

                void set_initial_position(
                        systems::Context<double> *context,
                        const Eigen::Ref<const VectorX<double>> x) const;

            private:
                void OutputCommand(const systems::Context<double> &context,
                                   systems::BasicVector<double> *output) const;

                void DoCalcDiscreteVariableUpdates(
                        const systems::Context<double> &context,
			const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
                        systems::DiscreteValues<double> *discrete_state) const override;

            private:
                const int num_joints_;
            };

        }  // namespace bhpn_drake_interface
    }  // namespace examples
}  // namespace drake
