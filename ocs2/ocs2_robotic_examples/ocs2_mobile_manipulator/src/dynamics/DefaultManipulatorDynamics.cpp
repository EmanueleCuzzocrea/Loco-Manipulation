#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DefaultManipulatorDynamics::DefaultManipulatorDynamics(const ManipulatorModelInfo& info, const std::string& modelName,
                                                       const std::string& modelFolder, bool recompileLibraries /*= true*/,
                                                       bool verbose /*= true*/) {
  this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t DefaultManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                      const ad_vector_t&) const {
  return input;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
