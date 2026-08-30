#ifndef GRAVITY_ALIGN_HPP
#define GRAVITY_ALIGN_HPP

// Gravity levelling helpers. Kept free of ROS so they can be unit tested.

#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "tools.hpp"

// Rotation taking `from` onto `to`. Well defined when the two are parallel or
// antiparallel, unlike an axis built from the cross product and divided by its
// own norm.
inline Eigen::Matrix3d rotationBetween(const Eigen::Vector3d &from,
                                       const Eigen::Vector3d &to)
{
  return Eigen::Quaterniond::FromTwoVectors(from, to).toRotationMatrix();
}

// Roll and pitch of a body at rest whose accelerometer reads `f_body`.
// A body at rest measures specific force R^T * (0,0,g), so with the
// Rz*Ry*Rx convention of makeRotationFromRollPitchYaw
//   f = (-g sin(pitch), g sin(roll) cos(pitch), g cos(roll) cos(pitch)).
// Yaw is not observable from gravity.
inline Eigen::Vector2d levelFromSpecificForce(const Eigen::Vector3d &f_body)
{
  const double roll = std::atan2(f_body.y(), f_body.z());
  const double pitch =
      std::atan2(-f_body.x(), std::hypot(f_body.y(), f_body.z()));
  return Eigen::Vector2d(roll, pitch);
}

struct StaticnessGate
{
  // Thresholds are on rms, not peak: a single bump or noise spike should not
  // veto an otherwise clean window. Measured on a stationary vehicle these sit
  // near 0.04 rad/s and 0.08-0.40 m/s^2, and while driving near 0.45 and 6.0,
  // so there is roughly an order of magnitude of separation either side.
  double gyro_rms_max = 0.15;     // rad/s, bias corrected
  double acc_norm_lo = 9.4;       // m/s^2
  double acc_norm_hi = 10.2;      // m/s^2
  double acc_dev_rms_max = 1.0;   // m/s^2, rms spread about the window mean
  double ba_norm_max = 0.5;       // m/s^2, reject an implausible bias estimate
  double speed_max = 0.05;        // m/s, see isQuasiStatic
  int min_samples = 100;
};

struct StaticnessStats
{
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
  double gyro_rms = 0.0;
  double acc_norm_mean = 0.0;
  double acc_dev_rms = 0.0;
  int samples = 0;
};

// Window statistics over matched accelerometer and gyroscope samples, both in
// the body frame and in SI units. `bg` is the current gyro bias estimate.
inline StaticnessStats accumulateStaticness(
    const std::vector<Eigen::Vector3d> &acc,
    const std::vector<Eigen::Vector3d> &gyr, const Eigen::Vector3d &bg)
{
  StaticnessStats s;
  const size_t n = std::min(acc.size(), gyr.size());
  if (n == 0)
    return s;
  s.samples = static_cast<int>(n);
  double gyro_sq = 0.0;
  for (size_t i = 0; i < n; i++)
  {
    s.mean_acc += acc[i];
    gyro_sq += (gyr[i] - bg).squaredNorm();
  }
  s.mean_acc /= static_cast<double>(n);
  s.acc_norm_mean = s.mean_acc.norm();
  s.gyro_rms = std::sqrt(gyro_sq / static_cast<double>(n));
  double acc_sq = 0.0;
  for (size_t i = 0; i < n; i++)
    acc_sq += (acc[i] - s.mean_acc).squaredNorm();
  s.acc_dev_rms = std::sqrt(acc_sq / static_cast<double>(n));
  return s;
}

// `speed` is the vehicle's own speed from an independent source. It is part of
// the test rather than an optional extra: the specific-force norm is nearly
// blind to horizontal acceleration. 2 m/s^2 laterally moves the norm by only
// 2%, well inside any usable band, while tilting the apparent vertical by
// 11.5 degrees. The gyro and scatter tests catch a turn or a bumpy ride, but
// steady acceleration in a straight line is only caught by knowing the vehicle
// is not moving.
inline bool isQuasiStatic(const StaticnessStats &s, const StaticnessGate &g,
                          const Eigen::Vector3d &ba, double speed)
{
  return s.samples >= g.min_samples && s.gyro_rms <= g.gyro_rms_max &&
         s.acc_norm_mean >= g.acc_norm_lo && s.acc_norm_mean <= g.acc_norm_hi &&
         s.acc_dev_rms <= g.acc_dev_rms_max && ba.norm() <= g.ba_norm_max &&
         std::abs(speed) <= g.speed_max;
}

// Rotation that takes the world frame the estimator is currently using onto one
// whose z axis is gravity. `R` is the current body->world rotation and `f_body`
// the bias-corrected specific force measured at the same instant.
inline Eigen::Matrix3d gravityDatumCorrection(const Eigen::Matrix3d &R,
                                              const Eigen::Vector3d &f_body)
{
  return rotationBetween(R * (-f_body), Eigen::Vector3d(0.0, 0.0, -1.0));
}

#endif // GRAVITY_ALIGN_HPP
