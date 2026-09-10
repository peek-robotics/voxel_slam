#ifndef VELOCITY_UPDATE_HPP
#define VELOCITY_UPDATE_HPP

// Post-IEKF scalar measurement update on a body-frame velocity component.
// Kept free of ROS so it can be unit tested.

#include "tools.hpp"

struct BodyVelocityUpdate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool applied = false;
  bool gated = false; // rejected by the innovation test
  Eigen::Matrix<double, DIM, 1> dx = Eigen::Matrix<double, DIM, 1>::Zero();
  Eigen::Matrix<double, DIM, DIM> cov = Eigen::Matrix<double, DIM, DIM>::Zero();
  double residual = 0.0;
  double innovation_var = 0.0; // H P H^T + meas_var, i.e. unboosted
  double nis = 0.0;
};

// Jacobian of h(x) = (R^T v)[axis] with respect to the error state
// [dtheta | p | v | bg | ba], for the right-multiplied rotation perturbation
// R <- R * Exp(dtheta) used by IMUST::operator+=.
//
//   d(R^T v)/d(dtheta) =  hat(R^T v)      -> row `axis`
//   d(R^T v)/dv        =  R^T             -> row `axis` is R.col(axis)
//
// Note the rotation block vanishes when the velocity is along the measured
// axis: a forward speed carries no attitude information.
inline Eigen::Matrix<double, 1, DIM> bodyVelocityJacobian(
    const Eigen::Matrix3d &R, const Eigen::Vector3d &v_world, int axis)
{
  Eigen::Vector3d v_body = R.transpose() * v_world;
  Eigen::Matrix<double, 1, DIM> H;
  H.setZero();
  H.block<1, 3>(0, 0) = hat(v_body).row(axis);
  H.block<1, 3>(0, 6) = R.col(axis).transpose();
  return H;
}

// axis: 0 = body x, 1 = body y. z is the measured component, meas_var its
// variance (already clamped by the caller), weight >= 1 the degeneracy gain
// boost; the effective measurement variance is meas_var / weight. The
// innovation test uses the unboosted variance, so raising the weight cannot
// tighten the gate. nis_max <= 0 disables the test.
inline BodyVelocityUpdate computeBodyVelocityUpdate(
    const Eigen::Matrix3d &R, const Eigen::Vector3d &v_world,
    const Eigen::Matrix<double, DIM, DIM> &P, int axis, double z,
    double meas_var, double weight, double nis_max)
{
  BodyVelocityUpdate out;

  Eigen::Vector3d v_body = R.transpose() * v_world;
  out.residual = v_body(axis) - z;

  Eigen::Matrix<double, 1, DIM> Hv = bodyVelocityJacobian(R, v_world, axis);
  Eigen::Matrix<double, DIM, 1> PHt = P * Hv.transpose();
  double HPHt = (Hv * PHt)[0];

  double S_gate = HPHt + meas_var;
  if (!std::isfinite(S_gate) || S_gate <= 0.0)
    return out;
  out.innovation_var = S_gate;
  out.nis = out.residual * out.residual / S_gate;
  if (nis_max > 0.0 && out.nis > nis_max)
  {
    out.gated = true;
    return out;
  }

  double R_eff = meas_var / weight;
  double S = HPHt + R_eff;
  if (!std::isfinite(S) || S <= 0.0)
    return out;

  Eigen::Matrix<double, DIM, 1> K = PHt / S;
  out.dx = -K * out.residual;

  // Joseph form: the short form (I - K H) P is exact only for the exact gain
  // and does not stay symmetric, and this covariance is inverted every frame.
  Eigen::Matrix<double, DIM, DIM> IKH =
      Eigen::Matrix<double, DIM, DIM>::Identity() - K * Hv;
  Eigen::Matrix<double, DIM, DIM> C =
      IKH * P * IKH.transpose() + K * R_eff * K.transpose();
  out.cov = 0.5 * (C + C.transpose());
  out.applied = true;
  return out;
}

#endif // VELOCITY_UPDATE_HPP
