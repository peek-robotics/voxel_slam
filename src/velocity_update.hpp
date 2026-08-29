#ifndef VELOCITY_UPDATE_HPP
#define VELOCITY_UPDATE_HPP

// Post-IEKF scalar measurement update on a body-frame velocity component.
// Kept free of ROS so it can be unit tested.

#include "tools.hpp"

struct BodyVelocityUpdate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool applied = false;
  Eigen::Matrix<double, DIM, 1> dx = Eigen::Matrix<double, DIM, 1>::Zero();
  Eigen::Matrix<double, DIM, DIM> cov = Eigen::Matrix<double, DIM, DIM>::Zero();
  double residual = 0.0;
  double innovation_var = 0.0;
};

// axis: 0 = body x, 1 = body y. z is the measured component, meas_var its
// variance (already clamped by the caller), weight >= 1 the degeneracy gain
// boost; the effective measurement variance is meas_var / weight.
inline BodyVelocityUpdate computeBodyVelocityUpdate(
    const Eigen::Matrix3d &R, const Eigen::Vector3d &v_world,
    const Eigen::Matrix<double, DIM, DIM> &P, int axis, double z,
    double meas_var, double weight)
{
  BodyVelocityUpdate out;

  Eigen::Vector3d v_body = R.transpose() * v_world;
  out.residual = v_body(axis) - z;

  Eigen::Matrix<double, 1, DIM> Hv;
  Hv.setZero();
  Hv(0, 0) = -v_body.y();
  Hv(0, 1) = v_body.x();
  Hv(0, 2) = 0.0;
  Hv(0, 6) = R(axis, 0);
  Hv(0, 7) = R(axis, 1);
  Hv(0, 8) = R(axis, 2);

  Eigen::Matrix<double, DIM, 1> PHt = P * Hv.transpose();
  double S = (Hv * PHt)[0] + meas_var / weight;
  if (!std::isfinite(S) || S <= 0.0)
    return out;
  out.innovation_var = S;

  Eigen::Matrix<double, DIM, 1> K = PHt / S;
  out.dx = K * out.residual;
  out.cov = (Eigen::Matrix<double, DIM, DIM>::Identity() - K * Hv) * P;
  out.applied = true;
  return out;
}

#endif // VELOCITY_UPDATE_HPP
