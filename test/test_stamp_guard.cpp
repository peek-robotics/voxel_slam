#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "stamp_guard.hpp"

namespace
{

constexpr double kImuDt = 0.005;
constexpr double kCloudDt = 0.1;
constexpr double kEpoch = 1.79e9;

// Feeds both streams at their nominal rates over [t0, t1) on one time base and
// counts what the guard said. `host` maps a stamp to the host clock at receipt.
struct Counts
{
  int accept = 0;
  int hold = 0;
  int step = 0;
};

Counts run(StampGuard& g, double t0, double t1, double host_offset = 0.0,
           double stamp_offset = 0.0, bool with_imu = true, bool with_cloud = true)
{
  Counts c;
  double next_cloud = t0;
  for (double t = t0; t < t1 - 1e-9; t += kImuDt)
  {
    std::vector<StampDecision> ds;
    if (with_imu)
      ds.push_back(g.observe(StampStream::Imu, t + stamp_offset, t + host_offset));
    if (with_cloud && t >= next_cloud - 1e-9)
    {
      ds.push_back(g.observe(StampStream::Cloud, next_cloud + stamp_offset,
                             next_cloud + host_offset));
      next_cloud += kCloudDt;
    }
    for (const StampDecision& d : ds)
    {
      if (d.verdict == StampVerdict::Accept)
        c.accept++;
      else if (d.verdict == StampVerdict::Hold)
        c.hold++;
      else
        c.step++;
    }
  }
  return c;
}

StampGuardConfig hostChecked()
{
  StampGuardConfig c;
  c.max_host_offset_s = 10.0;
  return c;
}

} // namespace

TEST(StampGuard, ContinuousStreamsAreAccepted)
{
  StampGuard g;
  const Counts c = run(g, 1000.0, 1010.0);
  EXPECT_EQ(0, c.step);
  EXPECT_EQ(0, c.hold);
  EXPECT_EQ(0u, g.epoch());
}

TEST(StampGuard, AMillisecondStepOnTheCloudStreamIsNotAStep)
{
  // A host-stamped cloud stream switching to the sensor clock once it syncs
  // moves by the transport latency, a few milliseconds either way.
  for (double shift : {0.005, -0.005})
  {
    StampGuard g(hostChecked());
    run(g, 1000.0, 1005.0, 0.0);
    double next_cloud = 1005.0;
    int steps = 0;
    for (double t = 1005.0; t < 1010.0; t += kImuDt)
    {
      steps += g.observe(StampStream::Imu, t, t).verdict != StampVerdict::Accept;
      if (t >= next_cloud - 1e-9)
      {
        steps += g.observe(StampStream::Cloud, next_cloud + shift, next_cloud).verdict !=
                 StampVerdict::Accept;
        next_cloud += kCloudDt;
      }
    }
    EXPECT_EQ(0, steps) << "shift " << shift;
    EXPECT_EQ(0u, g.epoch()) << "shift " << shift;
  }
}

TEST(StampGuard, AForwardEpochStepOnBothStreamsIsOneStep)
{
  StampGuard g;
  run(g, 64.0, 70.0);
  const Counts c = run(g, 70.0 + kEpoch, 75.0 + kEpoch);
  EXPECT_EQ(1, c.step);
  EXPECT_EQ(1u, g.epoch());
  // It settles on the new base and releases.
  EXPECT_FALSE(g.holding());
  EXPECT_EQ(1u, g.released());
  EXPECT_GT(c.accept, 0);
}

TEST(StampGuard, TheStepIsReportedOnTheFirstSteppedStamp)
{
  StampGuard g;
  run(g, 64.0, 70.0);
  const StampDecision d = g.observe(StampStream::Imu, 70.0 + kEpoch, 0.0);
  EXPECT_EQ(StampVerdict::Step, d.verdict);
  EXPECT_NEAR(kEpoch, d.dt, 1.0);
  EXPECT_FALSE(d.reason.empty());
}

TEST(StampGuard, NothingIsAcceptedUntilBothStreamsHaveSettled)
{
  StampGuard g;
  run(g, 64.0, 70.0);
  const double base = 70.0 + kEpoch;
  EXPECT_EQ(StampVerdict::Step, g.observe(StampStream::Imu, base, 0.0).verdict);
  // Well inside the settle period: still held.
  const Counts during = run(g, base + kImuDt, base + 0.9);
  EXPECT_EQ(0, during.accept);
  EXPECT_TRUE(g.holding());
  const Counts after = run(g, base + 0.9, base + 2.0);
  EXPECT_GT(after.accept, 0);
  EXPECT_FALSE(g.holding());
}

TEST(StampGuard, ABackwardEpochStepIsAStep)
{
  StampGuard g;
  run(g, 1000.0 + kEpoch, 1005.0 + kEpoch);
  const Counts c = run(g, 64.0, 70.0);
  EXPECT_EQ(1, c.step);
  EXPECT_EQ(1u, g.epoch());
  EXPECT_FALSE(g.holding());
}

TEST(StampGuard, StreamsThatStepMillisecondsApartAreOneEpisode)
{
  // The IMU reaches the new base first, the clouds one frame later.
  StampGuard g;
  run(g, 64.0, 70.0);
  int steps = 0;
  steps += g.observe(StampStream::Imu, 70.0 + kEpoch, 0.0).verdict == StampVerdict::Step;
  steps += g.observe(StampStream::Cloud, 70.0, 0.0).verdict == StampVerdict::Step;
  steps += g.observe(StampStream::Imu, 70.005 + kEpoch, 0.0).verdict == StampVerdict::Step;
  const Counts c = run(g, 70.1 + kEpoch, 75.0 + kEpoch);
  steps += c.step;
  EXPECT_EQ(1, steps);
  EXPECT_EQ(1u, g.epoch());
  EXPECT_FALSE(g.holding());
}

TEST(StampGuard, AClockThatFlapsWhileSettlingIsOneEpisode)
{
  StampGuard g;
  run(g, 64.0, 70.0);
  // Alternate between the two bases every 0.3 s, never long enough to settle.
  int steps = 0;
  for (int i = 0; i < 10; i++)
  {
    const double off = (i % 2 == 0) ? kEpoch : 0.0;
    steps += run(g, 70.0 + 0.3 * i, 70.3 + 0.3 * i, 0.0, off).step;
  }
  EXPECT_EQ(1, steps);
  EXPECT_TRUE(g.holding());
  run(g, 80.0 + kEpoch, 85.0 + kEpoch);
  EXPECT_FALSE(g.holding());
  EXPECT_EQ(1u, g.epoch());
}

TEST(StampGuard, SmallImuReorderingPassesThrough)
{
  // One sample delivered late, a few milliseconds behind the newest, then the
  // stream carries on.
  StampGuard g;
  run(g, 1000.0, 1001.0);
  const double newest = g.lastStamp(StampStream::Imu);
  EXPECT_EQ(StampVerdict::Accept,
            g.observe(StampStream::Imu, newest - 0.0095, newest).verdict);
  EXPECT_EQ(StampVerdict::Accept,
            g.observe(StampStream::Imu, newest + kImuDt, newest).verdict);
  EXPECT_EQ(0u, g.epoch());
}

TEST(StampGuard, ReorderingDoesNotWalkTheReferenceBackwards)
{
  // Each late sample is measured against the newest stamp, so two of them
  // cannot add up to a regression the tolerance would reject.
  StampGuard g;
  run(g, 1000.0, 1001.0);
  const double newest = g.lastStamp(StampStream::Imu);
  EXPECT_EQ(StampVerdict::Accept,
            g.observe(StampStream::Imu, newest - 0.045, newest).verdict);
  EXPECT_DOUBLE_EQ(newest, g.lastStamp(StampStream::Imu));
  EXPECT_EQ(StampVerdict::Step,
            g.observe(StampStream::Imu, newest - 0.09, newest).verdict);
}

TEST(StampGuard, EqualImuStampsPassThrough)
{
  StampGuard g;
  run(g, 1000.0, 1001.0);
  const double newest = g.lastStamp(StampStream::Imu);
  EXPECT_EQ(StampVerdict::Accept, g.observe(StampStream::Imu, newest, newest).verdict);
  EXPECT_EQ(0u, g.epoch());
}

TEST(StampGuard, AnImuStepBeyondTheReorderToleranceIsAStep)
{
  StampGuard g;
  run(g, 1000.0, 1001.0);
  const double newest = g.lastStamp(StampStream::Imu);
  EXPECT_EQ(StampVerdict::Step,
            g.observe(StampStream::Imu, newest - 0.06, newest).verdict);
}

TEST(StampGuard, AnImuOutageIsAStep)
{
  // IMU withheld while the sensor is unsynced, clouds carrying on: the clouds
  // drift off the IMU stream first, and nothing is used until the IMU returns
  // and settles.
  StampGuard g;
  run(g, 1000.0, 1005.0);
  const Counts outage = run(g, 1005.0, 1010.0, 0.0, 0.0, /*with_imu=*/false);
  EXPECT_EQ(1, outage.step);
  EXPECT_TRUE(g.holding());
  const Counts back = run(g, 1010.0, 1015.0);
  EXPECT_EQ(0, back.step);
  EXPECT_FALSE(g.holding());
  EXPECT_EQ(1u, g.epoch());
}

TEST(StampGuard, AnImuGapIsAStepEvenWithoutClouds)
{
  StampGuard g;
  run(g, 1000.0, 1002.0);
  EXPECT_EQ(StampVerdict::Step, g.observe(StampStream::Imu, 1003.0, 0.0).verdict);
}

TEST(StampGuard, ACloudDropoutWhileTheImuCarriesOnIsNotAStep)
{
  // By default: the IMU covers the interval, so there is nothing to reset.
  StampGuard g;
  run(g, 1000.0, 1002.0);
  run(g, 1002.0, 1002.8, 0.0, 0.0, /*with_imu=*/true, /*with_cloud=*/false);
  EXPECT_EQ(StampVerdict::Accept, g.observe(StampStream::Cloud, 1002.8, 0.0).verdict);
  EXPECT_EQ(0u, g.epoch());
}

TEST(StampGuard, TheCloudGapCheckFiresWhenConfigured)
{
  StampGuardConfig c;
  c.cloud_max_gap_s = 0.5;
  StampGuard g(c);
  run(g, 1000.0, 1002.0);
  const double last = g.lastStamp(StampStream::Cloud);
  EXPECT_EQ(StampVerdict::Accept,
            g.observe(StampStream::Cloud, last + 2 * kCloudDt, 0.0).verdict);
  run(g, 1002.0, 1002.8, 0.0, 0.0, /*with_imu=*/true, /*with_cloud=*/false);
  const StampDecision d = g.observe(StampStream::Cloud, 1002.8, 0.0);
  EXPECT_EQ(StampVerdict::Step, d.verdict);
  EXPECT_NEAR(0.7, d.dt, 1e-9);
}

TEST(StampGuard, ARepeatedCloudStampIsAStep)
{
  StampGuard g;
  run(g, 1000.0, 1002.0);
  const double last = g.lastStamp(StampStream::Cloud);
  EXPECT_EQ(StampVerdict::Step, g.observe(StampStream::Cloud, last, 0.0).verdict);
}

TEST(StampGuard, ACloudOffTheImuStreamIsAStep)
{
  // With the cloud's own gap check off, so only the cross-stream check can fire.
  StampGuardConfig c;
  c.cloud_max_gap_s = 0.0;
  StampGuard g(c);
  run(g, 1000.0, 1002.0);
  EXPECT_EQ(StampVerdict::Accept, g.observe(StampStream::Cloud, 1002.5, 0.0).verdict);
  EXPECT_EQ(StampVerdict::Step, g.observe(StampStream::Cloud, 1004.0, 0.0).verdict);
}

TEST(StampGuard, TheHostCheckHoldsAStreamThatStartsOnAForeignClock)
{
  // Uptime-stamped from the first message, so there is no previous stamp to
  // compare against: only the host clock can tell.
  StampGuard g(hostChecked());
  const Counts before = run(g, 64.0, 70.0, /*host_offset=*/kEpoch);
  EXPECT_EQ(0, before.accept);
  EXPECT_EQ(0, before.step); // nothing had been used, so nothing to reset
  EXPECT_TRUE(g.holding());
  const Counts after = run(g, 70.0 + kEpoch, 75.0 + kEpoch, 0.0);
  EXPECT_EQ(0, after.step);
  EXPECT_GT(after.accept, 0);
  EXPECT_FALSE(g.holding());
  EXPECT_EQ(0u, g.epoch());
}

TEST(StampGuard, WithoutTheHostCheckAForeignClockIsUsed)
{
  StampGuard g;
  const Counts c = run(g, 64.0, 70.0, kEpoch);
  EXPECT_EQ(0, c.hold);
  EXPECT_GT(c.accept, 0);
}

TEST(StampGuard, AnUnavailableHostClockSkipsTheHostCheck)
{
  // Simulated time is zero until the first clock message.
  StampGuard g(hostChecked());
  EXPECT_EQ(StampVerdict::Accept, g.observe(StampStream::Imu, 1000.0, 0.0).verdict);
}

TEST(StampGuard, LeavingTheHostClockMidRunIsAStep)
{
  StampGuard g(hostChecked());
  run(g, 1000.0, 1005.0);
  EXPECT_EQ(StampVerdict::Step,
            g.observe(StampStream::Imu, 1005.0, 1005.0 + 60.0).verdict);
}

TEST(StampGuard, ANonFiniteStampIsNeverAccepted)
{
  StampGuard g;
  run(g, 1000.0, 1001.0);
  EXPECT_EQ(StampVerdict::Step, g.observe(StampStream::Imu, NAN, 0.0).verdict);
}

TEST(StampGuard, DisabledChecksDoNotFire)
{
  StampGuardConfig c;
  c.imu_max_gap_s = 0.0;
  c.cloud_max_gap_s = 0.0;
  c.max_cross_stream_offset_s = 0.0;
  StampGuard g(c);
  run(g, 1000.0, 1002.0);
  EXPECT_EQ(StampVerdict::Accept, g.observe(StampStream::Imu, 1100.0, 0.0).verdict);
  EXPECT_EQ(StampVerdict::Accept, g.observe(StampStream::Cloud, 1100.0, 0.0).verdict);
}
