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
    double uncertain_orientation_variance = -1.0)
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
    p.pose_position_absolute = uncertain_variance;
    p.pose_orientation_absolute = uncertain_orientation_variance >= 0.0
                                          ? uncertain_orientation_variance
                                          : uncertain_variance;
    p.twist_absolute = uncertain_variance;
    p.zero_off_diagonal = true;
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

#endif // ODOM_PUBLISH_POLICY_HPP
