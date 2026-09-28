#ifndef SEED_PLAUSIBILITY_HPP
#define SEED_PLAUSIBILITY_HPP

// Decides whether a pose taken from the external odometry transform may seed
// the estimator. Kept free of ROS so it can be unit tested.
//
// A re-seed replaces the estimator's position with whatever the transform
// reports, so the transform's error becomes this estimator's error. If the
// transform is itself derived from this estimator's output, a single bad
// output can be read back as a seed and made permanent. The bounds here are
// what stops that: a seed must be finite, within a radius of the frame origin,
// and within reach of the last pose this estimator trusted.

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

// Mirrors the LIODiag SEED_* constants.
enum class SeedStatus : uint8_t
{
  None = 0,
  Accepted = 1,
  RejectedNonFinite = 2,
  RejectedRadius = 3,
  RejectedJump = 4,
  FallbackTrusted = 5,
  Withheld = 6
};

struct SeedBounds
{
  // Largest horizontal distance of a seed from the frame origin. Non-positive
  // disables the check.
  double max_radius_m = 0.0;
  // A seed may be this far from the last trusted pose, plus max_speed_mps for
  // every second since that pose. Non-positive max_jump_m disables the check.
  double max_jump_m = 0.0;
  double max_speed_mps = 0.0;
  // A trusted pose older than this is not used as a fallback seed.
  // Non-positive means it is never used.
  double max_trusted_age_s = 0.0;
};

// The last pose this estimator produced while healthy.
struct TrustedPose
{
  bool valid = false;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double time = 0.0;  // host clock
};

struct SeedVerdict
{
  SeedStatus status = SeedStatus::Accepted;
  // The bound that was exceeded and by how much, for logging.
  double value = 0.0;
  double limit = 0.0;
};

inline SeedVerdict checkSeed(const Eigen::Vector3d& p, double yaw,
                             const SeedBounds& b, const TrustedPose& trusted,
                             double now)
{
  SeedVerdict v;
  if (!p.allFinite() || !std::isfinite(yaw))
  {
    v.status = SeedStatus::RejectedNonFinite;
    return v;
  }

  if (b.max_radius_m > 0.0)
  {
    const double r = p.head<2>().norm();
    if (r > b.max_radius_m)
    {
      v.status = SeedStatus::RejectedRadius;
      v.value = r;
      v.limit = b.max_radius_m;
      return v;
    }
  }

  if (b.max_jump_m > 0.0 && trusted.valid)
  {
    // A clock that went backwards must not shrink the allowance below the
    // fixed part.
    const double elapsed = std::isfinite(now - trusted.time)
                                   ? std::max(0.0, now - trusted.time)
                                   : 0.0;
    const double limit = b.max_jump_m + std::max(0.0, b.max_speed_mps) * elapsed;
    const double jump = (p - trusted.p).head<2>().norm();
    if (!(jump <= limit))
    {
      v.status = SeedStatus::RejectedJump;
      v.value = jump;
      v.limit = limit;
      return v;
    }
  }

  return v;
}

// What to do when a seed is rejected: fall back to the trusted pose if it is
// recent enough, otherwise withhold the anchor altogether.
inline SeedStatus chooseFallback(const SeedBounds& b, const TrustedPose& trusted,
                                 double now)
{
  if (trusted.valid && b.max_trusted_age_s > 0.0 && trusted.p.allFinite())
  {
    const double age = now - trusted.time;
    if (std::isfinite(age) && age >= 0.0 && age <= b.max_trusted_age_s)
      return SeedStatus::FallbackTrusted;
  }
  return SeedStatus::Withheld;
}

inline const char* seedStatusName(SeedStatus s)
{
  switch (s)
  {
    case SeedStatus::None: return "none";
    case SeedStatus::Accepted: return "accepted";
    case SeedStatus::RejectedNonFinite: return "rejected (non-finite)";
    case SeedStatus::RejectedRadius: return "rejected (beyond radius)";
    case SeedStatus::RejectedJump: return "rejected (too far from last trusted pose)";
    case SeedStatus::FallbackTrusted: return "fell back to last trusted pose";
    case SeedStatus::Withheld: return "withheld";
  }
  return "unknown";
}

#endif // SEED_PLAUSIBILITY_HPP
