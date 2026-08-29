#include <gtest/gtest.h>

#include "odom_publish_policy.hpp"

namespace
{

const int kDiscFrames = 2;
const double kDiscVar = 1e6;
const double kUncVar = 1e3;

OdomPublishPolicy decide(bool anchored, bool initializing, DegradeState d,
                         int since)
{
  return decideOdomPublish(anchored, initializing, d, since, kDiscFrames,
                           kDiscVar, kUncVar);
}

const DegradeState kAllStates[] = {DegradeState::Ok, DegradeState::Low,
                                   DegradeState::Medium, DegradeState::High,
                                   DegradeState::Reset};

} // namespace

TEST(OdomPublishPolicy, NeverPublishesBeforeTheFirstAnchor)
{
  for (bool init : {false, true})
    for (DegradeState d : kAllStates)
      for (int since = 0; since < 5; ++since)
        EXPECT_FALSE(decide(false, init, d, since).publish);
}

TEST(OdomPublishPolicy, AlwaysPublishesOnceAnchored)
{
  for (bool init : {false, true})
    for (DegradeState d : kAllStates)
      for (int since = 0; since < 5; ++since)
        EXPECT_TRUE(decide(true, init, d, since).publish)
            << "init " << init << " degrade " << static_cast<int>(d)
            << " since " << since;
}

TEST(OdomPublishPolicy, DiscontinuityDominatesEveryOtherState)
{
  for (bool init : {false, true})
    for (DegradeState d : kAllStates)
      for (int since = 0; since < kDiscFrames; ++since)
      {
        const OdomPublishPolicy p = decide(true, init, d, since);
        EXPECT_EQ(p.pose_absolute, kDiscVar);
        EXPECT_EQ(p.twist_absolute, kDiscVar);
        EXPECT_TRUE(p.zero_off_diagonal);
      }
}

TEST(OdomPublishPolicy, InitialisingAndHighUseTheUncertainVariance)
{
  const OdomPublishPolicy a = decide(true, true, DegradeState::Ok, 99);
  EXPECT_EQ(a.pose_absolute, kUncVar);
  EXPECT_TRUE(a.zero_off_diagonal);

  const OdomPublishPolicy b = decide(true, false, DegradeState::High, 99);
  EXPECT_EQ(b.pose_absolute, kUncVar);
  EXPECT_TRUE(b.zero_off_diagonal);
}

TEST(OdomPublishPolicy, EffectiveVarianceIsMonotone)
{
  const double nominal = 1e-4;
  auto effective = [&](const OdomPublishPolicy &p) {
    return p.pose_absolute > 0.0 ? p.pose_absolute : nominal * p.pose_scale;
  };
  const double ok = effective(decide(true, false, DegradeState::Ok, 99));
  const double low = effective(decide(true, false, DegradeState::Low, 99));
  const double med = effective(decide(true, false, DegradeState::Medium, 99));
  const double high = effective(decide(true, false, DegradeState::High, 99));
  const double disc = effective(decide(true, false, DegradeState::Ok, 0));
  EXPECT_LT(ok, low);
  EXPECT_LT(low, med);
  EXPECT_LT(med, high);
  EXPECT_LT(high, disc);
}

TEST(OdomPublishPolicy, AbsoluteRegimeLeavesNoOffDiagonalTerms)
{
  double cov[36];
  for (int k = 0; k < 36; ++k)
    cov[k] = 7.0;
  const OdomPublishPolicy p = decide(true, true, DegradeState::Ok, 99);
  applyOdomCovariance(cov, p.pose_absolute, p.pose_scale, p.zero_off_diagonal);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      EXPECT_EQ(cov[i * 6 + j], i == j ? kUncVar : 0.0);
}

TEST(OdomPublishPolicy, ScaleRegimeMultipliesEverything)
{
  double cov[36];
  for (int k = 0; k < 36; ++k)
    cov[k] = 2.0;
  const OdomPublishPolicy p = decide(true, false, DegradeState::Low, 99);
  applyOdomCovariance(cov, p.pose_absolute, p.pose_scale, p.zero_off_diagonal);
  for (int k = 0; k < 36; ++k)
    EXPECT_EQ(cov[k], 2.0 * 1e3);
}

TEST(OdomPublishPolicy, DegradedTwistIsNotSilentlyScaled)
{
  // Low and Medium inflate the pose only; the twist is a local quantity and
  // stays as the filter reported it.
  const OdomPublishPolicy p = decide(true, false, DegradeState::Medium, 99);
  EXPECT_EQ(p.twist_scale, 1.0);
  EXPECT_EQ(p.twist_absolute, 0.0);
}
