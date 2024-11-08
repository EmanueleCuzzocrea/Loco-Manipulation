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
  dxdt[2] = -10*(state[2] - 0.3);
  dxdt[4] = -10*state[4];
  dxdt[5] = -10*state[5];

  dxdt[6] = -10*(state[6] - 0.0);
  dxdt[7] = -10*(state[7] - (-0.569));
  dxdt[8] = -10*(state[8] - 0.0);
  dxdt[9] = -10*(state[9] - (-2.810));
  dxdt[10] = -10*(state[10] - 0.0);
  dxdt[11] = -10*(state[11] - 3.037);
  dxdt[12] = -10*(state[12] - 0.741);
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
