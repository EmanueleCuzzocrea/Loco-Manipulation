#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <gtest/gtest.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_assets/package_path.h>

#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace mobile_manipulator;

class testEndEffectorConstraint : public ::testing::Test {
 public:
  using quaternion_t = EndEffectorConstraint::quaternion_t;
  using vector3_t = EndEffectorConstraint::vector3_t;

  testEndEffectorConstraint()
      : pinocchioInterface(createMobileManipulatorPinocchioInterface()),
        modelInfo(loadManipulatorModelInfo()),
        pinocchioMapping(modelInfo) {
    // initialize reference manager
    const vector_t positionOrientation = (vector_t(7) << vector3_t::Zero(), quaternion_t(1, 0, 0, 0).coeffs()).finished();
    referenceManagerPtr.reset(new ReferenceManager(TargetTrajectories({0.0}, {positionOrientation})));

    // initialize kinematics
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematics(pinocchioInterface, pinocchioMapping, {modelInfo.eeFrame}));
    preComputationPtr.reset(new MobileManipulatorPreComputation(pinocchioInterface, modelInfo));

    x.resize(modelInfo.stateDim);
    x << 1.0, 1.0, 0.5, 2.5, -1.0, 1.5, 0.0, 1.0, 0.0;
  }

  vector_t x;
  PinocchioInterface pinocchioInterface;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<MobileManipulatorPreComputation> preComputationPtr;
  std::shared_ptr<ReferenceManager> referenceManagerPtr;
  MobileManipulatorPinocchioMapping pinocchioMapping;
  ManipulatorModelInfo modelInfo;

 protected:
  ManipulatorModelInfo loadManipulatorModelInfo() {
    // files
    const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/mabi_mobile/task.info";
    // read the task file
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    // resolve meta-information about the model
    // read manipulator type
    ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
    // read the frame names
    std::string baseFrame, eeFrame;
    loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
    loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
    // return model
    return mobile_manipulator::createManipulatorModelInfo(pinocchioInterface, modelType, baseFrame, eeFrame);
  }

  PinocchioInterface createMobileManipulatorPinocchioInterface() {
    // files
    const std::string urdfPath = ocs2::robotic_assets::getPath() + "/resources/mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf";
    const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/mabi_mobile/task.info";
    // read manipulator type
    ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
    // read the joints to make fixed
    std::vector<std::string> removeJointNames;
    loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
    // initialize pinocchio interface
    return createPinocchioInterface(urdfPath, modelType, removeJointNames);
  }
};

TEST_F(testEndEffectorConstraint, testConstraintEvaluation) {
  EndEffectorConstraint eeConstraint(*eeKinematicsPtr, *referenceManagerPtr);

  auto& pinocchioInterface = preComputationPtr->getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = pinocchioMapping.getPinocchioJointPosition(x);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  std::cerr << "constraint:\n" << eeConstraint.getValue(0.0, x, *preComputationPtr) << '\n';
  std::cerr << "approximation:\n" << eeConstraint.getLinearApproximation(0.0, x, *preComputationPtr);
}
