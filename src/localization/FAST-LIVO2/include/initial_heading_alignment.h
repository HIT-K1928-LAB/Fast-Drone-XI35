#ifndef FAST_LIVO_INITIAL_HEADING_ALIGNMENT_H
#define FAST_LIVO_INITIAL_HEADING_ALIGNMENT_H

#include "body_pose_extrinsic.h"

#include <Eigen/Geometry>

#include <cmath>

namespace fast_livo
{

inline double yawFromRotation(const Eigen::Matrix3d &rotation_world_body)
{
  return std::atan2(rotation_world_body(1, 0), rotation_world_body(0, 0));
}

inline Eigen::Matrix3d worldRotationToZeroInitialBodyYaw(
    const Eigen::Matrix3d &rotation_world_body)
{
  return Eigen::AngleAxisd(
             -yawFromRotation(rotation_world_body),
             Eigen::Vector3d::UnitZ())
      .toRotationMatrix();
}

inline RigidBodyState rotateWorldFrame(
    const RigidBodyState &state,
    const Eigen::Matrix3d &rotation_aligned_world)
{
  RigidBodyState aligned;
  aligned.rotation = rotation_aligned_world * state.rotation;
  aligned.position = rotation_aligned_world * state.position;
  aligned.velocity = rotation_aligned_world * state.velocity;
  return aligned;
}

inline RigidBodyState alignWorldFrameAtInitialPosition(
    const RigidBodyState &state,
    const Eigen::Matrix3d &rotation_aligned_world,
    const Eigen::Vector3d &initial_position_world)
{
  RigidBodyState aligned;
  aligned.rotation = rotation_aligned_world * state.rotation;
  aligned.position = rotation_aligned_world *
      (state.position - initial_position_world);
  aligned.velocity = rotation_aligned_world * state.velocity;
  return aligned;
}

}  // namespace fast_livo

#endif
