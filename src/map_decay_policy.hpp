#ifndef MAP_DECAY_POLICY_HPP
#define MAP_DECAY_POLICY_HPP

// Decides when a voxel leaves the persistent voxel map, and when a voxel is
// confirmed enough to be matched against. Kept free of ROS and of the map
// types so it can be unit tested.
//
// This replaces a travelled-distance rule (evict when the vehicle has moved
// `d` metres since the voxel was last in the sliding window). That rule has
// three failure modes this one does not:
//   - a stationary vehicle never advances the distance counter, so nothing is
//     ever evicted while the map keeps growing from moving foliage and noise;
//   - the sweep only ran on the odometry thread's idle branch, behind a
//     release flag that is raised at most once every ten windows, so a thread
//     that keeps up with its input never evicts at all;
//   - the counter is zeroed on reset and on loop update, so any voxel that
//     outlives one of those is compared against a counter that restarted
//     behind it.
// Scan time has none of those properties: it advances while parked, it is
// available on every frame, and it is not rewound by a reset.
//
// `min_obs` is the separate half. A voxel seen in fewer than `min_obs`
// distinct scans stays in the map but is not matched against, so transient
// returns - people, vehicles, wind-moved canopy - cannot become pose
// constraints while they are still unconfirmed. They are promoted as soon as
// they accumulate enough observations, so slow-to-confirm structure is not
// lost the way an outright reject would lose it.

#include <cstdint>

struct MapDecayPolicy
{
  // [s] of scan time since a voxel was last observed, past which it is
  // evicted. <= 0 disables eviction (the map then grows without bound, which
  // is what offline reprocessing of a short bag wants).
  double decay_sec = 0.0;

  // Distinct scans a voxel must appear in before it may be matched against.
  // <= 1 matches every voxel, reproducing the previous behaviour exactly.
  int min_obs = 1;
};

// Whether a voxel last observed at `last_seen_t` should be evicted at scan
// time `now`.
//
// `exempt` marks voxels built from a loaded prior map or from a loop-closure
// map rebuild rather than from a live scan. Those carry no meaningful
// observation time and are precisely the map we mean to keep, so they never
// age out.
//
// A voxel that has never been observed (`last_seen_t` < 0) is never evicted:
// it has just been created and has not reached its first observe() yet.
//
// A clock that goes backwards - a looping bag, a reset that rewinds the scan
// clock - gives a negative age, which reads as "seen recently" rather than as
// an enormous age. A rewind therefore cannot flush the whole map in one sweep.
inline bool shouldEvictVoxel(double now, double last_seen_t, bool exempt,
                             const MapDecayPolicy& p)
{
  if (exempt || p.decay_sec <= 0.0 || last_seen_t < 0.0)
    return false;
  return (now - last_seen_t) > p.decay_sec;
}

// Whether a voxel observed in `obs_count` distinct scans may be used as a
// registration constraint.
inline bool voxelMatchable(uint32_t obs_count, bool exempt,
                           const MapDecayPolicy& p)
{
  if (exempt || p.min_obs <= 1)
    return true;
  return obs_count >= static_cast<uint32_t>(p.min_obs);
}

#endif // MAP_DECAY_POLICY_HPP
