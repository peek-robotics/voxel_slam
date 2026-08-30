#include <gtest/gtest.h>

#include "gravity_align.hpp"

namespace
{

Eigen::Matrix3d rpy(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// What an accelerometer on a body at rest with attitude R reads.
Eigen::Vector3d specificForce(const Eigen::Matrix3d &R, double g = G_m_s2)
{
  return R.transpose() * Eigen::Vector3d(0.0, 0.0, g);
}

double deg(double d) { return d * M_PI / 180.0; }

// Deterministic pseudo-noise, unit rms per axis.
Eigen::Vector3d noise(int i, int salt)
{
  const double a = std::sin(12.9898 * (i + salt) + 78.233);
  const double b = std::sin(39.4260 * (i + salt) + 11.135);
  const double c = std::sin(93.9898 * (i + salt) + 47.771);
  return Eigen::Vector3d(a, b, c) * 1.2247; // ~unit rms per axis
}

// Field-measured levels: on a stationary vehicle the gyro sits near 0.04 rad/s
// rms and the accelerometer spread near 0.08-0.40 m/s^2 rms; while driving they
// are near 0.45 and 6.0 respectively.
void fillWindow(std::vector<Eigen::Vector3d> &acc, std::vector<Eigen::Vector3d> &gyr,
                int n, double acc_rms, double gyro_rms, int salt = 0)
{
  acc.clear();
  gyr.clear();
  for (int i = 0; i < n; ++i)
  {
    acc.push_back(Eigen::Vector3d(0.0, 0.0, G_m_s2) + noise(i, salt) * acc_rms / std::sqrt(3.0));
    gyr.push_back(noise(i, salt + 977) * gyro_rms / std::sqrt(3.0));
  }
}

} // namespace

// The shipped align_gravity built its axis as (n0 x n1) / |n0 x n1|, which is
// 0/0 when the two are already parallel - the normal case, since the preserved
// gravity is vertical by construction.
TEST(RotationBetween, HandlesParallelAndAntiparallelInputs)
{
  const Eigen::Vector3d down(0.0, 0.0, -9.8);

  // The previous construction, for the record: with n0 already parallel to n1
  // the axis is 0/0 and the resulting matrix is entirely NaN, which would take
  // the whole state with it.
  {
    const Eigen::Vector3d n0 = down / down.norm();
    const Eigen::Vector3d n1(0, 0, -1);
    Eigen::Vector3d rotvec = n0.cross(n1);
    const double rnorm = rotvec.norm();
    rotvec = rotvec / rnorm;
    const Eigen::Matrix3d old = Eigen::AngleAxisd(std::asin(rnorm), rotvec).matrix();
    EXPECT_FALSE(old.allFinite());
  }

  const Eigen::Matrix3d same = rotationBetween(down, Eigen::Vector3d(0, 0, -1));
  ASSERT_TRUE(same.allFinite());
  EXPECT_LT((same - Eigen::Matrix3d::Identity()).norm(), 1e-12);

  const Eigen::Matrix3d flip = rotationBetween(down, Eigen::Vector3d(0, 0, 1));
  ASSERT_TRUE(flip.allFinite());
  EXPECT_LT((flip * Eigen::Vector3d(0, 0, -1) - Eigen::Vector3d(0, 0, 1)).norm(),
            1e-12);
}

TEST(LevelFromSpecificForce, RecoversAKnownTilt)
{
  for (int r = -30; r <= 30; r += 5)
    for (int p = -30; p <= 30; p += 5)
      for (double yaw : {0.0, 1.3, -2.7})
      {
        const Eigen::Matrix3d R = rpy(deg(r), deg(p), yaw);
        const Eigen::Vector2d rp = levelFromSpecificForce(specificForce(R));
        EXPECT_NEAR(rp.x(), deg(r), 1e-9) << "roll " << r << " pitch " << p;
        EXPECT_NEAR(rp.y(), deg(p), 1e-9) << "roll " << r << " pitch " << p;
      }
}

TEST(LevelFromSpecificForce, RemainsExactAtSteepTilts)
{
  for (double a : {85.0, -85.0})
  {
    Eigen::Vector2d rp = levelFromSpecificForce(specificForce(rpy(deg(a), 0, 0)));
    EXPECT_NEAR(rp.x(), deg(a), 1e-9);
    rp = levelFromSpecificForce(specificForce(rpy(0, deg(a), 0)));
    EXPECT_NEAR(rp.y(), deg(a), 1e-9);
  }
}

TEST(StaticnessGate, AcceptsARestingWindowAtFieldNoiseLevels)
{
  std::vector<Eigen::Vector3d> acc, gyr;
  // The noisier of the two stationary windows measured in the field.
  fillWindow(acc, gyr, 200, 0.40, 0.044);
  const StaticnessStats s = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
  EXPECT_TRUE(isQuasiStatic(s, StaticnessGate(), Eigen::Vector3d::Zero(), 0.0))
      << "gyro_rms " << s.gyro_rms << " acc_dev_rms " << s.acc_dev_rms
      << " |a| " << s.acc_norm_mean;
  EXPECT_EQ(s.samples, 200);
}

// Guards the threshold choice itself: the gate has to sit between the two
// levels actually observed, not below both.
TEST(StaticnessGate, SeparatesMeasuredStationaryFromMeasuredDriving)
{
  const StaticnessGate gate;
  std::vector<Eigen::Vector3d> acc, gyr;

  fillWindow(acc, gyr, 200, 0.079, 0.040, 1);
  StaticnessStats calm = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
  EXPECT_TRUE(isQuasiStatic(calm, gate, Eigen::Vector3d::Zero(), 0.0));

  fillWindow(acc, gyr, 200, 0.401, 0.044, 2);
  StaticnessStats bumpy = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
  EXPECT_TRUE(isQuasiStatic(bumpy, gate, Eigen::Vector3d::Zero(), 0.0));

  fillWindow(acc, gyr, 200, 6.011, 0.454, 3);
  StaticnessStats driving = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
  EXPECT_FALSE(isQuasiStatic(driving, gate, Eigen::Vector3d::Zero(), 0.0));

  // And with room to spare on both sides.
  EXPECT_LT(bumpy.gyro_rms * 2.0, driving.gyro_rms);
  EXPECT_LT(bumpy.acc_dev_rms * 2.0, driving.acc_dev_rms);
}

TEST(StaticnessGate, RejectsEachViolationIndependently)
{
  const StaticnessGate gate;
  std::vector<Eigen::Vector3d> acc, gyr;
  for (int i = 0; i < 200; ++i)
  {
    acc.push_back(Eigen::Vector3d(0.0, 0.0, G_m_s2));
    gyr.push_back(Eigen::Vector3d::Zero());
  }

  // Moving: rejected on speed alone, whatever the accelerometer says.
  {
    const StaticnessStats s = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
    EXPECT_FALSE(isQuasiStatic(s, gate, Eigen::Vector3d::Zero(), 1.2));
  }
  // Turning at a rate seen while driving.
  {
    std::vector<Eigen::Vector3d> w = gyr;
    for (auto &v : w)
      v.z() = 0.5;
    const StaticnessStats s = accumulateStaticness(acc, w, Eigen::Vector3d::Zero());
    EXPECT_FALSE(isQuasiStatic(s, gate, Eigen::Vector3d::Zero(), 0.0));
  }
  // Vibration at driving level: the mean is right but the samples are spread.
  {
    std::vector<Eigen::Vector3d> a = acc;
    for (size_t i = 0; i < a.size(); ++i)
      a[i].x() += (i % 2 ? 6.0 : -6.0);
    const StaticnessStats s = accumulateStaticness(a, gyr, Eigen::Vector3d::Zero());
    EXPECT_FALSE(isQuasiStatic(s, gate, Eigen::Vector3d::Zero(), 0.0));
  }
  // Too short.
  {
    std::vector<Eigen::Vector3d> a(acc.begin(), acc.begin() + 20);
    std::vector<Eigen::Vector3d> w(gyr.begin(), gyr.begin() + 20);
    const StaticnessStats s = accumulateStaticness(a, w, Eigen::Vector3d::Zero());
    EXPECT_FALSE(isQuasiStatic(s, gate, Eigen::Vector3d::Zero(), 0.0));
  }
  // Implausible accelerometer bias estimate.
  {
    const StaticnessStats s = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
    EXPECT_FALSE(isQuasiStatic(s, gate, Eigen::Vector3d(0.9, 0.0, 0.0), 0.0));
  }
}

// The reason speed is part of the test and not an optional extra.
TEST(StaticnessGate, NormAloneCannotSeeHorizontalAcceleration)
{
  std::vector<Eigen::Vector3d> acc, gyr;
  for (int i = 0; i < 200; ++i)
  {
    acc.push_back(Eigen::Vector3d(2.0, 0.0, G_m_s2)); // 2 m/s^2 forward
    gyr.push_back(Eigen::Vector3d::Zero());           // straight line
  }
  const StaticnessStats s = accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
  const StaticnessGate gate;

  // The norm barely moves, so every IMU-only test passes...
  EXPECT_GT(s.acc_norm_mean, gate.acc_norm_lo);
  EXPECT_LT(s.acc_norm_mean, gate.acc_norm_hi);
  EXPECT_LT(s.acc_dev_rms, gate.acc_dev_rms_max);
  EXPECT_LT(s.gyro_rms, gate.gyro_rms_max);

  // ...while the apparent vertical is tilted by more than 10 degrees.
  const double tilt = std::atan2(2.0, G_m_s2) * 180.0 / M_PI;
  EXPECT_GT(tilt, 10.0);

  // Only the speed catches it.
  EXPECT_TRUE(isQuasiStatic(s, gate, Eigen::Vector3d::Zero(), 0.0));
  EXPECT_FALSE(isQuasiStatic(s, gate, Eigen::Vector3d::Zero(), 1.0));
}

TEST(StaticnessGate, GyroBiasIsRemovedBeforeTheRateTest)
{
  std::vector<Eigen::Vector3d> acc, gyr;
  const Eigen::Vector3d bg(0.0, 0.0, 0.5);
  for (int i = 0; i < 200; ++i)
  {
    acc.push_back(Eigen::Vector3d(0.0, 0.0, G_m_s2));
    gyr.push_back(bg); // stationary, but the raw rate looks large
  }
  const StaticnessStats biased =
      accumulateStaticness(acc, gyr, Eigen::Vector3d::Zero());
  EXPECT_FALSE(isQuasiStatic(biased, StaticnessGate(), Eigen::Vector3d::Zero(), 0.0));
  const StaticnessStats corrected = accumulateStaticness(acc, gyr, bg);
  EXPECT_TRUE(isQuasiStatic(corrected, StaticnessGate(), Eigen::Vector3d::Zero(), 0.0));
}

// The estimator's world frame is tilted by C relative to gravity. The
// correction must undo exactly that, and must not touch yaw.
TEST(GravityDatumCorrection, UndoesAWorldFrameTiltAndLeavesYawAlone)
{
  for (double tilt_deg : {1.0, 4.0, 12.0})
    for (double yaw : {0.0, 2.0, -1.1})
    {
      const Eigen::Matrix3d R_true = rpy(deg(3.0), deg(-2.0), yaw);
      const Eigen::Matrix3d C =
          Eigen::AngleAxisd(deg(tilt_deg), Eigen::Vector3d::UnitX())
              .toRotationMatrix();
      const Eigen::Matrix3d R_est = C * R_true;

      const Eigen::Matrix3d corr =
          gravityDatumCorrection(R_est, specificForce(R_true));

      // Gravity comes back to vertical.
      EXPECT_LT((corr * C * Eigen::Vector3d(0, 0, -1) -
                 Eigen::Vector3d(0, 0, -1)).norm(),
                1e-12);

      // The corrected attitude has the true roll and pitch.
      const Eigen::Matrix3d R_fixed = corr * R_est;
      const Eigen::Vector3d f_fixed = specificForce(R_true);
      const Eigen::Vector2d want = levelFromSpecificForce(f_fixed);
      const Eigen::Vector2d got =
          levelFromSpecificForce(R_fixed.transpose() *
                                 Eigen::Vector3d(0, 0, G_m_s2));
      EXPECT_NEAR(got.x(), want.x(), 1e-9);
      EXPECT_NEAR(got.y(), want.y(), 1e-9);

      // The correction is a horizontal-axis rotation: gravity says nothing
      // about yaw.
      const Eigen::AngleAxisd aa(corr);
      if (aa.angle() > 1e-9)
        EXPECT_LT(std::abs(aa.axis().z()), 1e-9);
    }
}
