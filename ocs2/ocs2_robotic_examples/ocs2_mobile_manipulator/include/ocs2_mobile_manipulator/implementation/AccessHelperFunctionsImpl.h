#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // resolve the position vector based on robot type.
  switch (info.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, we assume robot is at identity pose
      return Eigen::Matrix<SCALAR, 3, 1>::Zero();
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // for floating arm, the first three entries correspond to base position
      return state.head(3);
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // for floating arm, the first three entries correspond to base position
      return state.head(3);
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, we assume 2D base position
      return Eigen::Matrix<SCALAR, 3, 1>(state(0), state(1), 0.0);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // resolve the position vector based on robot type.
  switch (info.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, we assume robot is at identity pose
      return Eigen::Quaternion<SCALAR>::Identity();
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // for floating arm, the base orientation is given by ZYX joints
      return ::ocs2::getQuaternionFromEulerAnglesZyx<SCALAR>(state.segment(3, 3));
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // for floating arm, the base orientation is given by ZYX joints
      return ::ocs2::getQuaternionFromEulerAnglesZyx<SCALAR>(state.segment(3, 3));
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, we assume only yaw
      return Eigen::Quaternion<SCALAR>(Eigen::AngleAxis<SCALAR>(state(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  const size_t startRow = info.stateDim - info.armDim;
  return Eigen::Block<Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // resolve for arm dof start index
  const size_t startRow = info.stateDim - info.armDim;
  return Eigen::Block<const Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

}  // namespace mobile_manipulator
}  // namespace ocs2
