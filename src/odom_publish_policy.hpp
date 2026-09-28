#ifndef ODOM_PUBLISH_POLICY_HPP
#define ODOM_PUBLISH_POLICY_HPP

// Decides whether an odometry message is publishable and how its covariance
// should be presented. Kept free of ROS so it can be unit tested.

#include "tools.hpp"

struct OdomPublishPolicy
{
  bool publish = false;
  // Absolute regimes replace the covariance diagonal outright and zero the
  // off-diagonals; a 6x6 block filled uniformly is singular, and a partly
  // filled one is indefinite.
  bool zero_off_diagonal = false;
  // Position and orientation take separate absolute variances. While
  // initialising, the position is re-seeded from the external transform on
  // every attempt whereas the orientation is carried over from the state being
  // reset, so the two are not uncertain to the same degree and one number
  // cannot describe both. Consumers differ as well: a filter may fuse the
  // position and ignore the orientation entirely.
  double pose_position_absolute = 0.0; // >0: replace the diagonal, else scale
  double pose_orientation_absolute = 0.0;
  double pose_scale = 1.0;
  double twist_absolute = 0.0;
  double twist_scale = 1.0;

  // Directional regime for the position block. When pose_position_directional
  // is > 0 the caller writes that block as
  //     pose_position_constrained * I + pose_position_directional * u u^T
  // for the unit vector u along which the LiDAR cannot see - see
  // applyDirectionalPositionCovariance(). pose_position_absolute is then 0, so
  // the absolute pass leaves the position diagonal alone and only zeroes the
  // off-diagonals (including the position-orientation cross terms) before this
  // block is written over them.
  //
  // Why it exists: in a crop row the LiDAR is blind along the row and fine
  // across it. Replacing the whole block with one number tells a downstream
  // filter the across-row estimate is as bad as the along-row one, so it
  // either throws away a good measurement or trusts a bad one. There is no
  // third number that is right for both.
  double pose_position_directional = 0.0;
  double pose_position_constrained = 0.0;
};

// A pose exists from the first anchor onwards, so publish from then on and let
// the covariance say how much to trust it. Before the first anchor there is
// nothing to publish. Known pose discontinuities (re-anchors, and the jump when
// an initialisation succeeds) are marked separately from degradation: a
// consumer differentiating the pose sees a step there, not a velocity.
// The orientation variances are negative by default, meaning "use the position
// value", which reproduces the single-variance behaviour exactly.
inline OdomPublishPolicy decideOdomPublish(
    bool anchored, bool initializing, DegradeState degrade,
    int frames_since_discontinuity, int discontinuity_frames,
    double discontinuity_variance, double uncertain_variance,
    double discontinuity_orientation_variance = -1.0,
    double uncertain_orientation_variance = -1.0,
    bool degeneracy_is_directional = false,
    double uncertain_constrained_variance = -1.0)
{
  OdomPublishPolicy p;
  if (!anchored)
    return p;
  p.publish = true;

  if (frames_since_discontinuity < discontinuity_frames)
  {
    p.pose_position_absolute = discontinuity_variance;
    p.pose_orientation_absolute = discontinuity_orientation_variance >= 0.0
                                          ? discontinuity_orientation_variance
                                          : discontinuity_variance;
    p.twist_absolute = discontinuity_variance;
    p.zero_off_diagonal = true;
    return p;
  }

  if (initializing || degrade >= DegradeState::High)
  {
    p.pose_orientation_absolute = uncertain_orientation_variance >= 0.0
                                          ? uncertain_orientation_variance
                                          : uncertain_variance;
    p.twist_absolute = uncertain_variance;
    p.zero_off_diagonal = true;

    // Only the degraded case can be answered directionally, and only when the
    // caller has established that the degeneracy really is one-sided.
    //
    // Initialisation is excluded on purpose: the pose is re-seeded from an
    // external transform there, so its error is that transform's and has
    // nothing to do with which way the LiDAR can see. Degradation reached
    // through the wheel-odometry watchdog rather than through LiDAR geometry
    // is excluded the same way, by the caller's `degeneracy_is_directional`
    // - claiming confidence across the row because the *LiDAR* is healthy
    // would be exactly wrong when the reason for distrust is elsewhere.
    const bool directional = !initializing && degeneracy_is_directional &&
                             uncertain_constrained_variance > 0.0 &&
                             uncertain_constrained_variance < uncertain_variance;
    if (directional)
    {
      p.pose_position_constrained = uncertain_constrained_variance;
      // So that the total along the weak direction is uncertain_variance,
      // i.e. the directional answer is never more pessimistic there than the
      // isotropic one it replaces.
      p.pose_position_directional =
              uncertain_variance - uncertain_constrained_variance;
    }
    else
    {
      p.pose_position_absolute = uncertain_variance;
    }
    return p;
  }

  if (degrade == DegradeState::Low)
    p.pose_scale = 1e3;
  else if (degrade == DegradeState::Medium)
    p.pose_scale = 1e6;
  return p;
}

// Applies one of the two regimes to a row-major 6x6 covariance array. The
// leading 3x3 block is linear (position, or linear velocity) and the trailing
// one is angular, so the absolute regime takes a value for each.
template <typename Cov>
inline void applyOdomCovariance(Cov &cov, double absolute_linear,
                                double absolute_angular, double scale,
                                bool zero_off_diagonal)
{
  if (absolute_linear > 0.0 || absolute_angular > 0.0)
  {
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
      {
        const double absolute = (i < 3) ? absolute_linear : absolute_angular;
        if (i == j)
        {
          // A non-positive value leaves that block's diagonal alone rather
          // than writing a zero variance, which would claim certainty.
          if (absolute > 0.0)
            cov[i * 6 + j] = absolute;
        }
        else if (zero_off_diagonal)
          cov[i * 6 + j] = 0.0;
      }
    return;
  }
  if (scale != 1.0)
    for (int k = 0; k < 36; ++k)
      cov[k] *= scale;
}

// Writes the 3x3 position block of a row-major 6x6 pose covariance as
//     constrained * I + directional * u u^T
// where u is `dir` normalised: `constrained` variance in every direction, plus
// `directional` more along u. Call it after applyOdomCovariance(), which has
// already zeroed the off-diagonals this overwrites.
//
// Both terms are positive semi-definite and `constrained` is required to be
// positive, so the result is positive definite by construction. That is the
// property the uniformly-filled block it replaces did not have: a 3x3 of one
// repeated value is rank 1, and a partly filled one is indefinite.
//
// A zero-length `dir` is treated as "no direction known" and yields the
// isotropic `constrained * I`, which is the safe reading of an absent input
// rather than a division by zero.
template <typename Cov>
inline void applyDirectionalPositionCovariance(Cov &cov,
                                               const Eigen::Vector3d &dir,
                                               double constrained,
                                               double directional)
{
  if (constrained <= 0.0)
    return;

  const double norm = dir.norm();
  const Eigen::Vector3d u = norm > 1e-9 ? Eigen::Vector3d(dir / norm)
                                        : Eigen::Vector3d::Zero();
  const double extra = directional > 0.0 ? directional : 0.0;

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      cov[i * 6 + j] = (i == j ? constrained : 0.0) + extra * u[i] * u[j];
}

#endif // ODOM_PUBLISH_POLICY_HPP
