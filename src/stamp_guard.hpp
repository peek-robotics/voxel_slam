#ifndef STAMP_GUARD_HPP
#define STAMP_GUARD_HPP

// Detects steps in the header stamps of the IMU and point cloud streams. Kept
// free of ROS so it can be unit tested.
//
// The estimator integrates the IMU between consecutive stamps. If a stream's
// clock steps - a sensor that starts on a free-running clock and later locks to
// a time source, a time source that is lost and regained, a driver restart -
// the next interval spans the step, and a step of years is integrated as if the
// vehicle had been moving for that long. Nothing downstream can tell such a
// pose from a real one.
//
// The checks are relative: each stamp against the previous one on its stream,
// clouds against the latest IMU stamp, and optionally either against the host
// clock. An absolute threshold on the stamp value would only catch a clock that
// happens to start near zero.
//
// After a step, input is held (dropped) until both streams have run cleanly on
// one time base for a settle period. A clock that steps several times while it
// settles - two streams that step a few milliseconds apart, a time source that
// flaps while it locks - is one episode and costs one reset, not one per step.

#include <cmath>
#include <cstdint>
#include <string>

enum class StampStream
{
  Imu = 0,
  Cloud = 1
};

struct StampGuardConfig
{
  // Largest forward gap between consecutive IMU stamps. Longer than this and
  // the interval is not something to dead-reckon across.
  double imu_max_gap_s = 0.5;
  // IMU samples can arrive slightly out of order; a sample this far behind the
  // newest one is passed through as before rather than treated as a step.
  double imu_max_backstep_s = 0.05;
  // Largest forward gap between consecutive cloud stamps. Off by default: a
  // gap in the clouds while the IMU carries on is a dropout, not a step, and
  // the IMU covers the interval. A cloud-only step forward is caught by the
  // cross-stream check. A cloud that does not advance at all is always a step.
  double cloud_max_gap_s = 0.0;
  // Largest offset between a cloud stamp and the newest IMU stamp.
  double max_cross_stream_offset_s = 1.0;
  // Largest offset between a stamp and the host clock. Only meaningful when
  // the sensors are stamped on the host's time base, so off by default.
  double max_host_offset_s = 0.0;
  // How long both streams must run cleanly before a hold is released.
  double settle_s = 1.0;
};

enum class StampVerdict
{
  // Use the message.
  Accept,
  // Drop the message; input is being held.
  Hold,
  // Drop the message; a step was detected on data that was being used. The
  // caller must discard everything buffered before it and reset.
  Step
};

struct StampDecision
{
  StampVerdict verdict = StampVerdict::Accept;
  // Why the stamp was not continuous, for logging. Empty when it was.
  std::string reason;
  // The interval that failed, where one applies.
  double dt = 0.0;
};

inline const char* stampStreamName(StampStream s)
{
  return s == StampStream::Imu ? "imu" : "cloud";
}

class StampGuard
{
public:
  explicit StampGuard(const StampGuardConfig& config = StampGuardConfig())
      : config_(config)
  {
  }

  // `host_now` is the host clock in the stamps' units; non-positive means it is
  // not available and the host check is skipped.
  StampDecision observe(StampStream stream, double t, double host_now)
  {
    StampDecision d;
    Track& self = track(stream);

    if (!std::isfinite(t))
    {
      d.reason = "non-finite stamp";
      return discontinuity(stream, t, d, /*restart_run=*/false);
    }

    if (config_.max_host_offset_s > 0.0 && host_now > 0.0 &&
        std::fabs(t - host_now) > config_.max_host_offset_s)
    {
      d.dt = t - host_now;
      d.reason = "stamp is off the host clock";
      // No run can start on a foreign time base.
      self.run_start = kNone;
      return discontinuity(stream, t, d, /*restart_run=*/false);
    }

    if (self.last != kNone)
    {
      const double dt = t - self.last;
      if (stream == StampStream::Imu)
      {
        if (dt < -config_.imu_max_backstep_s && config_.imu_max_backstep_s >= 0.0)
        {
          d.dt = dt;
          d.reason = "stamp went backwards";
          return discontinuity(stream, t, d, true);
        }
        if (dt < 0.0)
        {
          // Out of order, not a step. Keep measuring from the newest stamp.
          d.verdict = holding_ ? StampVerdict::Hold : StampVerdict::Accept;
          return d;
        }
        if (config_.imu_max_gap_s > 0.0 && dt > config_.imu_max_gap_s)
        {
          d.dt = dt;
          d.reason = "gap between stamps";
          return discontinuity(stream, t, d, true);
        }
      }
      else
      {
        if (dt <= 0.0)
        {
          d.dt = dt;
          d.reason = "stamp did not advance";
          return discontinuity(stream, t, d, true);
        }
        if (config_.cloud_max_gap_s > 0.0 && dt > config_.cloud_max_gap_s)
        {
          d.dt = dt;
          d.reason = "gap between stamps";
          return discontinuity(stream, t, d, true);
        }
      }
    }

    if (stream == StampStream::Cloud && config_.max_cross_stream_offset_s > 0.0 &&
        imu_.last != kNone && std::fabs(t - imu_.last) > config_.max_cross_stream_offset_s)
    {
      d.dt = t - imu_.last;
      d.reason = "stamp is off the imu stream";
      return discontinuity(stream, t, d, true);
    }

    self.last = t;
    if (self.run_start == kNone)
      self.run_start = t;

    if (holding_ && settled())
    {
      holding_ = false;
      released_++;
    }

    if (holding_)
    {
      d.verdict = StampVerdict::Hold;
      return d;
    }
    live_ = true;
    d.verdict = StampVerdict::Accept;
    return d;
  }

  bool holding() const { return holding_; }
  // Incremented on every step that invalidates buffered data.
  uint64_t epoch() const { return epoch_; }
  // Holds released so far.
  uint64_t released() const { return released_; }
  double lastStamp(StampStream s) const
  {
    const Track& tr = s == StampStream::Imu ? imu_ : cloud_;
    return tr.last;
  }
  const StampGuardConfig& config() const { return config_; }

private:
  static constexpr double kNone = -1e300;

  struct Track
  {
    double last = kNone;
    double run_start = kNone;
  };

  Track& track(StampStream s) { return s == StampStream::Imu ? imu_ : cloud_; }

  bool runSettled(const Track& tr) const
  {
    return tr.run_start != kNone && tr.last != kNone &&
           tr.last - tr.run_start >= config_.settle_s;
  }

  bool settled() const
  {
    if (!runSettled(imu_) || !runSettled(cloud_))
      return false;
    if (config_.max_cross_stream_offset_s > 0.0 &&
        std::fabs(cloud_.last - imu_.last) > config_.max_cross_stream_offset_s)
      return false;
    return true;
  }

  StampDecision discontinuity(StampStream stream, double t, StampDecision d,
                              bool restart_run)
  {
    Track& self = track(stream);
    if (restart_run)
    {
      // The new stamp starts this stream's run on whatever base it is now on.
      self.last = t;
      self.run_start = t;
    }

    if (live_)
    {
      // Data on the old base has been used; everything buffered is suspect,
      // and the other stream must show it is on the new base too.
      live_ = false;
      holding_ = true;
      epoch_++;
      Track& other = track(stream == StampStream::Imu ? StampStream::Cloud
                                                     : StampStream::Imu);
      other.run_start = kNone;
      d.verdict = StampVerdict::Step;
      return d;
    }

    // Nothing has been used since the last step, or at all: keep holding.
    holding_ = true;
    d.verdict = StampVerdict::Hold;
    return d;
  }

  StampGuardConfig config_;
  Track imu_;
  Track cloud_;
  bool holding_ = false;
  bool live_ = false;
  uint64_t epoch_ = 0;
  uint64_t released_ = 0;
};

#endif // STAMP_GUARD_HPP
