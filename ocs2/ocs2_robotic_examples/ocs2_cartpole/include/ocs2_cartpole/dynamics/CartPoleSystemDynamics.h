#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_cartpole/CartPoleParameters.h"
#include "ocs2_cartpole/definitions.h"

namespace ocs2 {
namespace cartpole {

/**
 * CartPole dynamics.
 * refer to: https://pdfs.semanticscholar.org/f95b/9d4cc0814034f2e601cb91fcd70b2e806420.pdf
 */
class CartPoleSytemDynamics : public SystemDynamicsBaseAD {
 public:
  CartPoleSytemDynamics(const CartPoleParameters& cartPoleParameters, const std::string& libraryFolder, bool verbose)
      : param_(cartPoleParameters) {
    initialize(STATE_DIM, INPUT_DIM, "cartpole_dynamics", libraryFolder, true, verbose);
  }

  ~CartPoleSytemDynamics() override = default;

  CartPoleSytemDynamics(const CartPoleSytemDynamics& rhs) = default;

  CartPoleSytemDynamics* clone() const override { return new CartPoleSytemDynamics(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override {
    const ad_scalar_t cosTheta = cos(state(0));
    const ad_scalar_t sinTheta = sin(state(0));

    // Inertia tensor
    Eigen::Matrix<ad_scalar_t, 2, 2> I;
    I << static_cast<ad_scalar_t>(param_.poleSteinerMoi_),
         static_cast<ad_scalar_t>(param_.poleMass_ * param_.poleLength_ * cosTheta),
         static_cast<ad_scalar_t>(param_.poleMass_ * param_.poleLength_ * cosTheta),
         static_cast<ad_scalar_t>(param_.cartMass_ + param_.poleMass_);

    // RHS
    Eigen::Matrix<ad_scalar_t, 2, 1> rhs(-param_.poleMass_ * param_.poleLength_ * param_.gravity_ * sinTheta,
                                         input(0) + param_.poleMass_ * param_.poleLength_ * pow(state(2), 2) * sinTheta);

    // dxdt
    ad_vector_t stateDerivative(STATE_DIM);
    stateDerivative << state.tail<2>(), I.inverse() * rhs;
    return stateDerivative;
  }

 private:
  CartPoleParameters param_;
};

}  // namespace cartpole
}  // namespace ocs2
