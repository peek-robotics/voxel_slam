#include <gtest/gtest.h>

#include <cmath>

#include "seed_plausibility.hpp"

namespace
{

SeedBounds bounds()
{
  SeedBounds b;
  b.max_radius_m = 10000.0;
  b.max_jump_m = 50.0;
  b.max_speed_mps = 5.0;
  b.max_trusted_age_s = 30.0;
  return b;
}

TrustedPose trustedAt(double x, double y, double time)
{
  TrustedPose t;
  t.valid = true;
  t.p = Eigen::Vector3d(x, y, 0.0);
  t.time = time;
  return t;
}

} // namespace

TEST(SeedPlausibility, ANearbySeedIsAccepted)
{
  const SeedVerdict v =
      checkSeed(Eigen::Vector3d(102.0, -40.0, 1.0), 0.3, bounds(), trustedAt(100.0, -41.0, 99.0), 100.0);
  EXPECT_EQ(SeedStatus::Accepted, v.status);
}

TEST(SeedPlausibility, ASeedFromAPoisonedTransformIsRejectedByRadius)
{
  // A transform that has integrated a clock step sits hundreds of km out.
  const SeedVerdict v = checkSeed(Eigen::Vector3d(-20378.1, -250159.0, 1.07), 0.0,
                                  bounds(), TrustedPose(), 100.0);
  EXPECT_EQ(SeedStatus::RejectedRadius, v.status);
  EXPECT_GT(v.value, v.limit);
}

TEST(SeedPlausibility, ANonFiniteSeedIsAlwaysRejected)
{
  SeedBounds off;  // every bound disabled
  EXPECT_EQ(SeedStatus::RejectedNonFinite,
            checkSeed(Eigen::Vector3d(NAN, 0, 0), 0.0, off, TrustedPose(), 0.0).status);
  EXPECT_EQ(SeedStatus::RejectedNonFinite,
            checkSeed(Eigen::Vector3d(INFINITY, 0, 0), 0.0, off, TrustedPose(), 0.0).status);
  EXPECT_EQ(SeedStatus::RejectedNonFinite,
            checkSeed(Eigen::Vector3d(0, 0, 0), NAN, off, TrustedPose(), 0.0).status);
}

TEST(SeedPlausibility, DisabledBoundsAcceptAnyFiniteSeed)
{
  SeedBounds off;
  EXPECT_EQ(SeedStatus::Accepted,
            checkSeed(Eigen::Vector3d(1e12, 1e12, 0), 0.0, off, trustedAt(0, 0, 0), 1.0).status);
}

TEST(SeedPlausibility, ASeedWithinTheRadiusButFarFromTheTrustedPoseIsRejected)
{
  const SeedVerdict v =
      checkSeed(Eigen::Vector3d(900.0, 0.0, 0.0), 0.0, bounds(), trustedAt(0.0, 0.0, 99.0), 100.0);
  EXPECT_EQ(SeedStatus::RejectedJump, v.status);
  EXPECT_DOUBLE_EQ(55.0, v.limit);
}

TEST(SeedPlausibility, TheJumpAllowanceGrowsWithTimeSinceTheTrustedPose)
{
  const Eigen::Vector3d seed(200.0, 0.0, 0.0);
  EXPECT_EQ(SeedStatus::RejectedJump,
            checkSeed(seed, 0.0, bounds(), trustedAt(0, 0, 90.0), 100.0).status);
  EXPECT_EQ(SeedStatus::Accepted,
            checkSeed(seed, 0.0, bounds(), trustedAt(0, 0, 60.0), 100.0).status);
}

TEST(SeedPlausibility, AClockGoingBackwardsDoesNotShrinkTheAllowance)
{
  EXPECT_EQ(SeedStatus::Accepted,
            checkSeed(Eigen::Vector3d(40.0, 0, 0), 0.0, bounds(), trustedAt(0, 0, 100.0), 50.0).status);
}

TEST(SeedPlausibility, WithoutATrustedPoseOnlyTheRadiusApplies)
{
  EXPECT_EQ(SeedStatus::Accepted,
            checkSeed(Eigen::Vector3d(5000.0, 0, 0), 0.0, bounds(), TrustedPose(), 100.0).status);
}

TEST(SeedPlausibility, HeightIsNotBoundedByTheRadius)
{
  EXPECT_EQ(SeedStatus::Accepted,
            checkSeed(Eigen::Vector3d(0, 0, 20000.0), 0.0, bounds(), TrustedPose(), 100.0).status);
}

TEST(SeedPlausibility, ARecentTrustedPoseIsTheFallback)
{
  EXPECT_EQ(SeedStatus::FallbackTrusted, chooseFallback(bounds(), trustedAt(1, 2, 90.0), 100.0));
}

TEST(SeedPlausibility, AStaleOrMissingTrustedPoseMeansWithhold)
{
  EXPECT_EQ(SeedStatus::Withheld, chooseFallback(bounds(), trustedAt(1, 2, 50.0), 100.0));
  EXPECT_EQ(SeedStatus::Withheld, chooseFallback(bounds(), TrustedPose(), 100.0));
  SeedBounds no_fallback = bounds();
  no_fallback.max_trusted_age_s = 0.0;
  EXPECT_EQ(SeedStatus::Withheld, chooseFallback(no_fallback, trustedAt(1, 2, 99.0), 100.0));
}

TEST(SeedPlausibility, ATrustedPoseFromTheFutureIsNotUsed)
{
  // A clock that stepped backwards makes the age negative; its meaning is lost.
  EXPECT_EQ(SeedStatus::Withheld, chooseFallback(bounds(), trustedAt(1, 2, 200.0), 100.0));
}
