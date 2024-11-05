#pragma once

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_python_interface/PythonInterface.h>

#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorPyBindings final : public PythonInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF of the robot.
   */
  MobileManipulatorPyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile) {
    // System dimensions
    stateDim_ = static_cast<int>(7);
    inputDim_ = static_cast<int>(7);

    // Robot interface
    MobileManipulatorInterface mobileManipulatorInterface(taskFile, libraryFolder, urdfFile);

    // MPC
    auto mpcPtr = std::make_unique<GaussNewtonDDP_MPC>(
        mobileManipulatorInterface.mpcSettings(), mobileManipulatorInterface.ddpSettings(), mobileManipulatorInterface.getRollout(),
        mobileManipulatorInterface.getOptimalControlProblem(), mobileManipulatorInterface.getInitializer());
    mpcPtr->getSolverPtr()->setReferenceManager(mobileManipulatorInterface.getReferenceManagerPtr());

    // Python interface
    PythonInterface::init(mobileManipulatorInterface, std::move(mpcPtr));
  }
};

}  // namespace mobile_manipulator
}  // namespace ocs2
