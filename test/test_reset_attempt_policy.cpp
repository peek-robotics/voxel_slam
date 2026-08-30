#include <gtest/gtest.h>

#include <vector>

#include "reset_attempt_policy.hpp"

namespace
{

ResetAttemptPolicy defaultPolicy()
{
  ResetAttemptPolicy p;
  p.escalate_after_s = 5.0;
  p.escalate_growth = 2.0;
  p.max_escalate_s = 60.0;
  return p;
}

} // namespace

TEST(ResetAttemptPolicy, EscalatesWhenNothingHasBeenResetYet)
{
  // No prior full reset means there is no interval to measure against, so the
  // first failure must not be deferred.
  const ResetAttemptDecision d = decideResetAttempt(defaultPolicy(), 0, 100.0, 0.0);
  EXPECT_TRUE(d.full_reset);
}

TEST(ResetAttemptPolicy, ARetrySoonAfterAFullResetClearsTheWindowInstead)
{
  // This is the defect: the previous behaviour ran a full reset here, one per
  // failed attempt, at the rate the window refills.
  const ResetAttemptDecision d = decideResetAttempt(defaultPolicy(), 0, 101.0, 100.0);
  EXPECT_FALSE(d.full_reset);
  EXPECT_DOUBLE_EQ(5.0, d.escalate_interval_s);
}

TEST(ResetAttemptPolicy, EscalatesOnceTheIntervalHasElapsed)
{
  EXPECT_TRUE(decideResetAttempt(defaultPolicy(), 0, 105.0, 100.0).full_reset);
  EXPECT_FALSE(decideResetAttempt(defaultPolicy(), 0, 104.999, 100.0).full_reset);
}

TEST(ResetAttemptPolicy, EachEscalationInTheSameStormWaitsLonger)
{
  const ResetAttemptPolicy p = defaultPolicy();
  EXPECT_DOUBLE_EQ(5.0, decideResetAttempt(p, 0, 200.0, 100.0).escalate_interval_s);
  EXPECT_DOUBLE_EQ(10.0, decideResetAttempt(p, 1, 200.0, 100.0).escalate_interval_s);
  EXPECT_DOUBLE_EQ(20.0, decideResetAttempt(p, 2, 200.0, 100.0).escalate_interval_s);

  // ...and the growth actually defers: 6 s after the first escalation is not
  // enough for the second.
  EXPECT_FALSE(decideResetAttempt(p, 1, 106.0, 100.0).full_reset);
  EXPECT_TRUE(decideResetAttempt(p, 1, 110.0, 100.0).full_reset);
}

TEST(ResetAttemptPolicy, TheIntervalIsCapped)
{
  const ResetAttemptPolicy p = defaultPolicy();
  EXPECT_DOUBLE_EQ(60.0, decideResetAttempt(p, 20, 1e6, 100.0).escalate_interval_s);
  // Including where the growth would otherwise overflow to infinity.
  EXPECT_DOUBLE_EQ(60.0, decideResetAttempt(p, 5000, 1e6, 100.0).escalate_interval_s);
}

TEST(ResetAttemptPolicy, GrowthAtOrBelowOneKeepsTheIntervalConstant)
{
  ResetAttemptPolicy p = defaultPolicy();
  p.escalate_growth = 1.0;
  EXPECT_DOUBLE_EQ(5.0, decideResetAttempt(p, 7, 200.0, 100.0).escalate_interval_s);
}

TEST(ResetAttemptPolicy, ANonPositiveIntervalRestoresAResetPerAttempt)
{
  ResetAttemptPolicy p = defaultPolicy();
  p.escalate_after_s = 0.0;
  EXPECT_TRUE(decideResetAttempt(p, 0, 100.001, 100.0).full_reset);
  EXPECT_TRUE(decideResetAttempt(p, 9, 100.001, 100.0).full_reset);
}

TEST(ResetAttemptPolicy, AClockThatHasNotAdvancedDoesNotEscalate)
{
  EXPECT_FALSE(decideResetAttempt(defaultPolicy(), 0, 100.0, 100.0).full_reset);
}

TEST(ResetAttemptPolicy, AClockThatWentBackwardsDoesNotEscalate)
{
  // The scan clock is re-seeded across a reset; a negative elapsed must not
  // read as "long enough".
  EXPECT_FALSE(decideResetAttempt(defaultPolicy(), 0, 90.0, 100.0).full_reset);
}

TEST(ResetAttemptPolicy, TheRecordedStormsCollapseToOneResetEach)
{
  // Reset timestamps observed on one recording, grouped by the watchdog
  // trigger that opened each storm. The first entry of each group is the
  // reset the trigger itself performed; the rest were init-failure retries,
  // each of which ran a full reset under the previous behaviour.
  const std::vector<std::vector<double>> storms = {
      {1854.5, 1855.4},
      {3397.0, 3397.9, 3398.9, 3399.9, 3400.9, 3401.9},
      {3408.7, 3409.6},
      {3416.1, 3417.0, 3418.0},
      {3768.3, 3769.2, 3770.2, 3771.2},
  };

  const ResetAttemptPolicy p = defaultPolicy();
  int full_resets = 0;
  for (const std::vector<double> &storm : storms)
  {
    // The trigger's own reset.
    double last_full = storm.front();
    int escalations = 0;
    full_resets++;

    for (size_t i = 1; i < storm.size(); i++)
    {
      const ResetAttemptDecision d =
          decideResetAttempt(p, escalations, storm[i], last_full);
      if (d.full_reset)
      {
        full_resets++;
        escalations++;
        last_full = storm[i];
      }
    }
  }

  // 17 resets from 5 triggers before; one per trigger after.
  EXPECT_EQ(5, full_resets);
}
