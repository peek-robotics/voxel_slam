#include "map_decay_policy.hpp"

#include <gtest/gtest.h>


namespace
{

MapDecayPolicy policy(double decay_sec)
{
  MapDecayPolicy p;
  p.decay_sec = decay_sec;
  return p;
}

} // namespace

// --- eviction --------------------------------------------------------------

TEST(MapDecayPolicy, EvictsOnlyPastTheThreshold)
{
  const MapDecayPolicy p = policy(300.0);
  EXPECT_FALSE(shouldEvictVoxel(1000.0, 701.0, p)); // 299 s old
  EXPECT_FALSE(shouldEvictVoxel(1000.0, 700.0, p)); // exactly 300 s
  EXPECT_TRUE(shouldEvictVoxel(1000.0, 699.0, p));  // 301 s old
}

TEST(MapDecayPolicy, DisabledByNonPositiveThreshold)
{
  // 0 and negative both mean "never evict" - what offline reprocessing of a
  // recording wants, and the escape hatch if decay is ever suspected of harm.
  EXPECT_FALSE(shouldEvictVoxel(1e6, 0.0, policy(0.0)));
  EXPECT_FALSE(shouldEvictVoxel(1e6, 0.0, policy(-1.0)));
}

TEST(MapDecayPolicy, NeverObservedVoxelIsNotEvicted)
{
  // last_seen_t < 0 is a voxel created this instant that has not reached its
  // first observe() yet. Treating the sentinel as an age would delete it
  // before it was ever used.
  EXPECT_FALSE(shouldEvictVoxel(1000.0, -1.0, policy(300.0)));
}

TEST(MapDecayPolicy, ClockGoingBackwardsDoesNotFlushTheMap)
{
  // Looped log playback, or a reset that rewinds the scan clock, makes `now` far
  // smaller than the stamps already in the map. That must read as "seen
  // recently", not as an enormous age - otherwise one sweep deletes
  // everything and the LIO has no map to match against.
  EXPECT_FALSE(shouldEvictVoxel(10.0, 5000.0, policy(300.0)));
}

TEST(MapDecayPolicy, StationaryVehicleStillAges)
{
  // The regression this whole change exists for: the previous rule keyed on
  // travelled distance, so a parked vehicle observing moving objects evicted
  // nothing while the map kept growing. Scan time advances regardless.
  const MapDecayPolicy p = policy(300.0);
  const double stamped_while_parked = 100.0;
  EXPECT_TRUE(shouldEvictVoxel(stamped_while_parked + 301.0,
                               stamped_while_parked, p));
}

