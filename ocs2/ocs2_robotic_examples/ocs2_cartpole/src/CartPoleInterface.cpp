#include <iostream>
#include <string>

#include "ocs2_cartpole/CartPoleInterface.h"
#include "ocs2_cartpole/dynamics/CartPoleSystemDynamics.h"

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace cartpole {

/* Task.info:
    initialState
    x_final
    ddp
    mpc

    Q
    R
    Q_final
    cartpole_parameters
    rollout
    bounds_penalty_config
*/
CartPoleInterface::CartPoleInterface(const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
  
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[CartPoleInterface] Loading task file: " << taskFilePath << "\n";
  } else {
    throw std::invalid_argument("[CartPoleInterface] Task file not found: " + taskFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[CartPoleInterface] Generated library path: " << libraryFolderPath << "\n";


  // Default initial condition
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
  if (verbose) {
    std::cerr << "x_init:   " << initialState_.transpose() << "\n";
    std::cerr << "x_final:  " << xFinal_.transpose() << "\n";
  }

  // DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);


  // ReferenceManager & SolverSynchronizedModule
  referenceManagerPtr_.reset(new ReferenceManager);


  

  // OPTIMAL CONTROL PROBLEM

  // Cost
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
  if (verbose) {
    std::cerr << "Q:  \n" << Q << "\n";
    std::cerr << "R:  \n" << R << "\n";
    std::cerr << "Q_final:\n" << Qf << "\n";
  }
  problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
  problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

  // Dynamics
  CartPoleParameters cartPoleParameters;
  cartPoleParameters.loadSettings(taskFile, "cartpole_parameters", verbose);
  problem_.dynamicsPtr.reset(new CartPoleSytemDynamics(cartPoleParameters, libraryFolder, verbose));

  // Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Constraints
  auto getPenalty = [&]() {
    // one can use either augmented::SlacknessSquaredHingePenalty or augmented::ModifiedRelaxedBarrierPenalty
    using penalty_type = augmented::SlacknessSquaredHingePenalty;
    penalty_type::Config boundsConfig;
    loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, verbose);
    return penalty_type::create(boundsConfig);
  };
  auto getConstraint = [&]() {
    constexpr size_t numIneqConstraint = 2;
    const vector_t e = (vector_t(numIneqConstraint) << cartPoleParameters.maxInput_, cartPoleParameters.maxInput_).finished();
    const vector_t D = (vector_t(numIneqConstraint) << 1.0, -1.0).finished();
    const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
    return std::make_unique<LinearStateInputConstraint>(e, C, D);
  };
  problem_.inequalityLagrangianPtr->add("InputLimits", create(getConstraint(), getPenalty()));

  // Initialization
  cartPoleInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

}  // namespace cartpole
}  // namespace ocs2
