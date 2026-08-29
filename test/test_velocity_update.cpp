#include <gtest/gtest.h>

#include "velocity_update.hpp"

namespace
{

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

// h(x) evaluated at the state perturbed by the error-state increment, using
// the same retraction as IMUST::operator+= (R <- R * Exp(dtheta), v <- v + dv).
double measurementAt(const Eigen::Matrix3d &R, const Eigen::Vector3d &v_world,
                     int axis, const Eigen::Matrix<double, DIM, 1> &d)
{
  Eigen::Matrix3d Rp = R * Exp(Eigen::Vector3d(d.segment<3>(0)));
  Eigen::Vector3d vp = v_world + d.segment<3>(6);
  return (Rp.transpose() * vp)(axis);
}

double bodyComponent(const Eigen::Matrix3d &R, const Eigen::Vector3d &v_world,
                     int axis)
{
  return (R.transpose() * v_world)(axis);
}

} // namespace

// The Jacobian is checked against a central difference taken through the
// filter's own retraction, so a convention mismatch cannot pass.
TEST(BodyVelocityJacobian, MatchesNumericalDerivative)
{
  const double eps = 1e-7;
  for (int axis = 0; axis < 2; ++axis)
    for (double yaw : {0.0, 0.9, -2.1, 2.8})
    {
      const Eigen::Matrix3d R = rpy(0.05, -0.03, yaw);
      const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.2, 0.1, -0.05);
      const Eigen::Matrix<double, 1, DIM> H =
          bodyVelocityJacobian(R, v_world, axis);

      for (int i = 0; i < DIM; ++i)
      {
        Eigen::Matrix<double, DIM, 1> d = Eigen::Matrix<double, DIM, 1>::Zero();
        d(i) = eps;
        const double up = measurementAt(R, v_world, axis, d);
        d(i) = -eps;
        const double dn = measurementAt(R, v_world, axis, d);
        EXPECT_NEAR(H(0, i), (up - dn) / (2.0 * eps), 1e-6)
            << "axis " << axis << " yaw " << yaw << " coord " << i;
      }
    }
}

// Restatement of the same fact in a form that needs no finite differences.
TEST(BodyVelocityJacobian, ForwardSpeedCarriesNoAttitudeInformation)
{
  const Eigen::Matrix3d R = rpy(0.02, -0.01, 0.7);
  const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.4, 0.0, 0.0);
  const Eigen::Matrix<double, 1, DIM> H = bodyVelocityJacobian(R, v_world, 0);
  const double attitude_block_norm = H.block<1, 3>(0, 0).norm();
  EXPECT_LT(attitude_block_norm, 1e-12);
}

TEST(BodyVelocityJacobian, LateralAxisUsesItsOwnRow)
{
  const Eigen::Matrix3d R = rpy(0.02, -0.01, 0.4);
  const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.0, 0.3, 0.0);
  const Eigen::Matrix<double, 1, DIM> H = bodyVelocityJacobian(R, v_world, 1);
  const Eigen::Vector3d v_body = R.transpose() * v_world;
  EXPECT_LT((H.block<1, 3>(0, 0) - hat(v_body).row(1)).norm(), 1e-12);
  EXPECT_LT((H.block<1, 3>(0, 6) - R.col(1).transpose()).norm(), 1e-12);
}

TEST(BodyVelocityUpdate, CorrectionMovesTowardTheMeasurement)
{
  const Eigen::Matrix3d R = rpy(0.0, 0.0, 0.5);
  const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.3, 0.0, 0.0);
  const double z = 0.77;

  const BodyVelocityUpdate u = computeBodyVelocityUpdate(
      R, v_world, samplePrior(), 0, z, 0.02, 1.0, 0.0);
  ASSERT_TRUE(u.applied);

  const Eigen::Vector3d v_after = v_world + u.dx.segment<3>(6);
  const double before = bodyComponent(R, v_world, 0);
  const double after = bodyComponent(R, v_after, 0);
  EXPECT_LT(after, before);
  EXPECT_GT(after, z);
}

// The velocity Jacobian block is R.col(axis); using R.row(axis) instead scales
// the gain by cos(2*yaw), so contraction holds only at some headings.
TEST(BodyVelocityUpdate, InnovationContractsAtEveryHeading)
{
  const Eigen::Matrix<double, DIM, DIM> P = samplePrior();
  for (int k = 0; k < 8; ++k)
  {
    const double yaw = k * M_PI / 4.0;
    const Eigen::Matrix3d R = rpy(0.0, 0.0, yaw);
    const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.3, 0.0, 0.0);
    const double z = 0.77;

    const BodyVelocityUpdate u =
        computeBodyVelocityUpdate(R, v_world, P, 0, z, 0.02, 1.0, 0.0);
    ASSERT_TRUE(u.applied) << "yaw index " << k;

    const Eigen::Matrix3d R_after = R * Exp(Eigen::Vector3d(u.dx.segment<3>(0)));
    const Eigen::Vector3d v_after = v_world + u.dx.segment<3>(6);
    const double r_after = bodyComponent(R_after, v_after, 0) - z;
    EXPECT_LT(std::abs(r_after), std::abs(u.residual)) << "yaw index " << k;
  }
}

TEST(BodyVelocityUpdate, CovarianceStaysSymmetricAndPositiveDefinite)
{
  const Eigen::Matrix3d R = rpy(0.01, 0.02, 1.1);
  Eigen::Matrix<double, DIM, DIM> P = samplePrior();
  Eigen::Vector3d v_world = R * Eigen::Vector3d(1.0, 0.0, 0.0);

  double prev_trace = P.trace();
  for (int i = 0; i < 1000; ++i)
  {
    const double z = 1.0 + 0.05 * std::sin(0.37 * i);
    const BodyVelocityUpdate u =
        computeBodyVelocityUpdate(R, v_world, P, 0, z, 0.02, 1.0, 0.0);
    ASSERT_TRUE(u.applied) << "iteration " << i;
    P = u.cov;
    v_world += u.dx.segment<3>(6);

    ASSERT_EQ(P, P.transpose()) << "iteration " << i;
    const Eigen::LDLT<Eigen::Matrix<double, DIM, DIM>> ldlt(P);
    ASSERT_TRUE(ldlt.isPositive()) << "iteration " << i;
    ASSERT_LE(P.trace(), prev_trace + 1e-12) << "iteration " << i;
    prev_trace = P.trace();
  }
}

TEST(BodyVelocityUpdate, InnovationTestRejectsAnOutlier)
{
  const Eigen::Matrix3d R = rpy(0.0, 0.0, 0.3);
  const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Matrix<double, DIM, DIM> P = samplePrior();

  const BodyVelocityUpdate u = computeBodyVelocityUpdate(
      R, v_world, P, 0, bodyComponent(R, v_world, 0) + 50.0, 0.02, 1.0, 25.0);
  EXPECT_FALSE(u.applied);
  EXPECT_TRUE(u.gated);
  EXPECT_TRUE(u.dx.isZero());
  EXPECT_TRUE(u.cov.isZero());
}

TEST(BodyVelocityUpdate, WeightBoostsTheGainButNotTheGate)
{
  const Eigen::Matrix3d R = rpy(0.0, 0.0, 0.3);
  const Eigen::Vector3d v_world = R * Eigen::Vector3d(1.3, 0.0, 0.0);
  const Eigen::Matrix<double, DIM, DIM> P = samplePrior();

  const BodyVelocityUpdate a =
      computeBodyVelocityUpdate(R, v_world, P, 0, 0.77, 0.02, 1.0, 0.0);
  const BodyVelocityUpdate b =
      computeBodyVelocityUpdate(R, v_world, P, 0, 0.77, 0.02, 5.0, 0.0);
  ASSERT_TRUE(a.applied);
  ASSERT_TRUE(b.applied);

  EXPECT_GT(b.dx.norm(), a.dx.norm());
  EXPECT_EQ(a.nis, b.nis);
  EXPECT_EQ(a.innovation_var, b.innovation_var);
}

TEST(BodyVelocityUpdate, RejectsNonPositiveInnovationVariance)
{
  Eigen::Matrix<double, DIM, DIM> P;
  P.setZero();
  const BodyVelocityUpdate u = computeBodyVelocityUpdate(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0), P, 0, 0.5,
      0.0, 1.0, 0.0);
  EXPECT_FALSE(u.applied);
  EXPECT_TRUE(u.dx.isZero());
}
