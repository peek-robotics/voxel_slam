// Golden test for the extraction of the body-velocity update out of
// VOXEL_SLAM. reference() below is a verbatim copy of the arithmetic as it
// stood inline. The extracted function must reproduce it; the effective
// measurement variance is now meas_var / weight rather than 1 / (weight /
// meas_var), which is the same value up to one ulp, so the comparison is to
// within rounding rather than exact.

#include <gtest/gtest.h>

#include "velocity_update.hpp"

namespace
{

BodyVelocityUpdate reference(const Eigen::Matrix3d &R,
                             const Eigen::Vector3d &v_world,
                             const Eigen::Matrix<double, DIM, DIM> &P, int axis,
                             double z, double meas_var, double weight)
{
  BodyVelocityUpdate out;
  const Eigen::Vector3d v_body = R.transpose() * v_world;
  const double v_body_x = v_body.x();
  const double v_body_y = v_body.y();
  const double r = v_body(axis) - z;
  const double R_inv = weight / meas_var;

  Eigen::Matrix<double, 1, DIM> Hv;
  Hv.setZero();
  Hv(0, 0) = -v_body_y;
  Hv(0, 1) = v_body_x;
  Hv(0, 2) = 0.0;
  Hv(0, 6) = R(axis, 0);
  Hv(0, 7) = R(axis, 1);
  Hv(0, 8) = R(axis, 2);

  const Eigen::Matrix<double, DIM, 1> PHt = P * Hv.transpose();
  const double S = (Hv * PHt)[0] + 1.0 / R_inv;
  if (!std::isfinite(S) || S <= 0.0)
    return out;
  const Eigen::Matrix<double, DIM, 1> K = PHt / S;
  out.residual = r;
  out.innovation_var = S;
  out.dx = K * r;
  out.cov = (Eigen::Matrix<double, DIM, DIM>::Identity() - K * Hv) * P;
  out.applied = true;
  return out;
}

Eigen::Matrix3d rpy(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Matrix<double, DIM, DIM> samplePrior()
{
  Eigen::Matrix<double, DIM, DIM> P;
  P.setIdentity();
  P *= 1e-2;
  P.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 1e-3;
  P(0, 6) = P(6, 0) = 1e-4;
  P(1, 7) = P(7, 1) = 2e-4;
  return P;
}

} // namespace

TEST(BodyVelocityUpdate, MatchesReferenceArithmetic)
{
  const Eigen::Matrix<double, DIM, DIM> P = samplePrior();
  const double yaws[] = {0.0, 0.6, 1.9, -2.4, 3.0};
  const double speeds[] = {0.3, 1.1, 2.0};
  const double weights[] = {1.0, 1.0000001, 3.0, 5.0};

  for (int axis = 0; axis < 2; ++axis)
    for (double yaw : yaws)
      for (double s : speeds)
      for (double w : weights)
      {
        const Eigen::Matrix3d R = rpy(0.03, -0.02, yaw);
        const Eigen::Vector3d v_world = R * Eigen::Vector3d(s, 0.05, -0.01);
        const double z = s - 0.4;

        const BodyVelocityUpdate got =
            computeBodyVelocityUpdate(R, v_world, P, axis, z, 0.05, w);
        const BodyVelocityUpdate want = reference(R, v_world, P, axis, z, 0.05, w);

        ASSERT_TRUE(got.applied);
        ASSERT_TRUE(want.applied);
        EXPECT_EQ(got.residual, want.residual);
        EXPECT_DOUBLE_EQ(got.innovation_var, want.innovation_var);
        EXPECT_LT((got.dx - want.dx).cwiseAbs().maxCoeff(), 1e-15);
        EXPECT_LT((got.cov - want.cov).cwiseAbs().maxCoeff(), 1e-15);
      }
}

TEST(BodyVelocityUpdate, RejectsNonPositiveInnovationVariance)
{
  Eigen::Matrix<double, DIM, DIM> P;
  P.setZero();
  const BodyVelocityUpdate got = computeBodyVelocityUpdate(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0), P, 0, 0.5,
      0.0, 1.0);
  EXPECT_FALSE(got.applied);
  EXPECT_TRUE(got.dx.isZero());
}
