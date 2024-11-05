#include "ocs2_mobile_manipulator/dynamics/FloatingArmManipulatorDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FloatingArmManipulatorDynamics::FloatingArmManipulatorDynamics(const ManipulatorModelInfo& info, const std::string& modelName,
                                                               const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                               bool recompileLibraries /*= true*/, bool verbose /*= true*/) {
  this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t FloatingArmManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                          const ad_vector_t&) const {
  ad_vector_t dxdt = ad_vector_t::Zero(state.size());
  dxdt.tail(input.size()) = input;  // only arm joint state
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
