#include "ocs2_mobile_manipulator/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FullyActuatedFloatingArmManipulatorDynamics::FullyActuatedFloatingArmManipulatorDynamics(const ManipulatorModelInfo& info,
                                                                                         const std::string& modelName,
                                                                                         const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                                                         bool recompileLibraries /*= true*/,
                                                                                         bool verbose /*= true*/) {
  this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t FullyActuatedFloatingArmManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                                       const ad_vector_t&) const {
  ad_vector_t dxdt = ad_vector_t::Zero(input.size());
  dxdt = input;
  dxdt[2] = -10*(state[2] - 0.6);
  dxdt[4] = -10*state[4];
  dxdt[5] = -10*state[5];
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
