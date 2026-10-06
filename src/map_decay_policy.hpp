#ifndef MAP_DECAY_POLICY_HPP
#define MAP_DECAY_POLICY_HPP

// Decides when a voxel leaves the persistent voxel map. Kept free of ROS and
// of the map types so it can be unit tested.
//
// This replaces a travelled-distance rule (evict when the vehicle has moved
// `d` metres since the voxel was last in the sliding window). That rule has
// three failure modes this one does not:
//   - a stationary vehicle never advances the distance counter, so nothing is
//     ever evicted while the map keeps growing from moving objects and noise;
//   - the sweep only ran on the odometry thread's idle branch, behind a
//     release flag that is raised at most once every ten windows, so a thread
//     that keeps up with its input never evicts at all;
//   - the counter is zeroed on reset and on loop update, so any voxel that
//     outlives one of those is compared against a counter that restarted
//     behind it.
// Scan time has none of those properties: it advances while parked, it is
// available on every frame, and it is not rewound by a reset. It is also not
// derived from the pose estimate, so a degenerate, drifting LIO does not
// evict its own map on travel that never happened.

struct MapDecayPolicy
{
  // [s] of scan time since a voxel was last observed, past which it is
  // evicted. <= 0 disables eviction (the map then grows without bound, which
  // is what offline reprocessing of a short recording wants).
  double decay_sec = 0.0;
};

// Whether a voxel last observed at `last_seen_t` should be evicted at scan
// time `now`.
//
// A voxel that has never been observed (`last_seen_t` < 0) is never evicted:
// it has just been created and has not reached its first observe() yet.
//
// A clock that goes backwards - looped log playback, a reset that rewinds the scan
// clock - gives a negative age, which reads as "seen recently" rather than as
// an enormous age. A rewind therefore cannot flush the whole map in one sweep.
inline bool shouldEvictVoxel(double now, double last_seen_t,
                             const MapDecayPolicy& p)
{
  if (p.decay_sec <= 0.0 || last_seen_t < 0.0)
    return false;
  return (now - last_seen_t) > p.decay_sec;
}

// One eviction pass over a voxel map keyed by root voxel. Returns the number
// of entries removed.
//
// `map` is any associative container whose mapped values point at a node
// with a `last_seen_t` member. `in_use(key)` vetoes eviction of a voxel the
// caller still depends on (the sliding window); `retire(node)` takes
// ownership of an evicted node before it leaves the map. The pass only
// erases - freeing is the caller's business, so it can be deferred.
template <typename Map, typename InUse, typename Retire>
int sweepDecayedVoxels(Map& map, double now, const MapDecayPolicy& p,
                       InUse&& in_use, Retire&& retire)
{
  if (p.decay_sec <= 0.0 || now <= 0.0)
    return 0;

  int evicted = 0;
  for (auto iter = map.begin(); iter != map.end();)
  {
    if (!shouldEvictVoxel(now, iter->second->last_seen_t, p) ||
        in_use(iter->first))
    {
      ++iter;
      continue;
    }
    retire(iter->second);
    iter = map.erase(iter);
    evicted++;
  }
  return evicted;
}

#endif // MAP_DECAY_POLICY_HPP
