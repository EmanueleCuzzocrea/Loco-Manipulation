#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Implementation of dynamics for a free floating arm manipulator with actuatable base.
 *
 * The floating base manipulator has the state: (6-DOF base pose, arm joints).
 * The base orientation is represented using zyx euler angles.
 * The end-effector targets are assumed to given with respect to the world frame.
 * The arm is assumed to be velocity controlled.
 */
class FullyActuatedFloatingArmManipulatorDynamics final : public SystemDynamicsBaseAD {
 public:
  /**
   * Constructor
   *
   * @param [in] modelInfo : The manipulator information.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose : Display information.
   */
  FullyActuatedFloatingArmManipulatorDynamics(const ManipulatorModelInfo& modelInfo, const std::string& modelName,
                                              const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                                              bool verbose = true);

  ~FullyActuatedFloatingArmManipulatorDynamics() override = default;
  FullyActuatedFloatingArmManipulatorDynamics* clone() const override { return new FullyActuatedFloatingArmManipulatorDynamics(*this); }

 private:
  FullyActuatedFloatingArmManipulatorDynamics(const FullyActuatedFloatingArmManipulatorDynamics& rhs) = default;

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& /*parameters*/) const override;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
