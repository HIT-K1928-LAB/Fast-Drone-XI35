#pragma once

#include <Eigen/Core>

namespace fast_livo
{

struct RigidBodyState
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

class BodyPoseExtrinsic
{
public:
  BodyPoseExtrinsic() = default;

  // Kalibr multi-IMU T_i_b convention:
  // p_target = R_target_reference * p_reference + t_target_reference.
  static BodyPoseExtrinsic FromTargetReference(
      const Eigen::Matrix3d &rotation_target_reference,
      const Eigen::Vector3d &translation_target_reference)
  {
    BodyPoseExtrinsic extrinsic;
    extrinsic.rotation_reference_target_ =
        rotation_target_reference.transpose();
    extrinsic.translation_reference_target_ =
        -extrinsic.rotation_reference_target_ * translation_target_reference;
    return extrinsic;
  }

  RigidBodyState transform(
      const RigidBodyState &reference_state,
      const Eigen::Vector3d &angular_velocity_reference =
          Eigen::Vector3d::Zero()) const
  {
    RigidBodyState target_state;
    target_state.rotation =
        reference_state.rotation * rotation_reference_target_;
    target_state.position =
        reference_state.position +
        reference_state.rotation * translation_reference_target_;
    target_state.velocity =
        reference_state.velocity + reference_state.rotation *
        angular_velocity_reference.cross(translation_reference_target_);
    return target_state;
  }

  const Eigen::Matrix3d &rotationReferenceTarget() const
  {
    return rotation_reference_target_;
  }

  const Eigen::Vector3d &translationReferenceTarget() const
  {
    return translation_reference_target_;
  }

private:
  Eigen::Matrix3d rotation_reference_target_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_reference_target_ = Eigen::Vector3d::Zero();
};

}  // namespace fast_livo
