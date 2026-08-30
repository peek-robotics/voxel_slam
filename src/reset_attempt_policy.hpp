#ifndef RESET_ATTEMPT_POLICY_HPP
#define RESET_ATTEMPT_POLICY_HPP

// How often a repeatedly failing initialisation is allowed to escalate to a
// full system reset. Kept free of ROS so it can be unit tested.
//
// A failed initialisation attempt is an expected outcome of trying to
// initialise where the geometry is momentarily poor, not a system fault. It
// needs the estimation window cleared and another attempt, which is cheap.
// A full reset is not cheap: it re-levels attitude, spending a gravity datum
// that may have taken minutes at rest to earn, and it counts as a reset
// everywhere downstream. Running one per failed attempt turns a single
// excursion into a storm of them.

#include <cmath>

// What a reset is allowed to do. A retry needs the estimation window cleared
// and the pose re-seeded so the next attempt starts from something sane; it
// must not re-level attitude, because the levelling datum is spent when it is
// applied, nor count as a reset downstream.
enum class ResetKind
{
  Full,
  InitRetry
};

struct ResetAttemptPolicy
{
  // Continuous failure tolerated before the first escalation to a full reset.
  // Non-positive escalates on every attempt, which is the behaviour this
  // policy replaces.
  double escalate_after_s = 5.0;
  // Each further escalation in the same storm waits this much longer, so a
  // situation that is not recovering stops thrashing.
  double escalate_growth = 2.0;
  double max_escalate_s = 60.0;
};

struct ResetAttemptDecision
{
  // Run the full reset, rather than clearing the window and retrying.
  bool full_reset = false;
  // The interval that was applied, for logging and diagnostics.
  double escalate_interval_s = 0.0;
};

// `escalations_so_far` counts full resets already spent on the current storm.
// `now` and `last_full_reset_time` share one clock; a non-positive
// `last_full_reset_time` means no full reset has happened yet.
inline ResetAttemptDecision decideResetAttempt(const ResetAttemptPolicy &p,
                                               int escalations_so_far,
                                               double now,
                                               double last_full_reset_time)
{
  ResetAttemptDecision d;

  if (!(p.escalate_after_s > 0.0))
  {
    d.full_reset = true;
    return d;
  }

  double interval = p.escalate_after_s;
  if (p.escalate_growth > 1.0 && escalations_so_far > 0)
  {
    interval *= std::pow(p.escalate_growth, static_cast<double>(escalations_so_far));
    if (!std::isfinite(interval))
      interval = p.max_escalate_s;
  }
  if (p.max_escalate_s > 0.0 && interval > p.max_escalate_s)
    interval = p.max_escalate_s;
  d.escalate_interval_s = interval;

  if (!(last_full_reset_time > 0.0))
  {
    // Nothing to measure an interval against yet.
    d.full_reset = true;
    return d;
  }

  const double elapsed = now - last_full_reset_time;
  // A clock that has not advanced, or has gone backwards across a reset, must
  // not be read as "the interval elapsed".
  if (!std::isfinite(elapsed))
    return d;

  d.full_reset = elapsed >= interval;
  return d;
}

#endif // RESET_ATTEMPT_POLICY_HPP
