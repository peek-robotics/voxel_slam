#include "map_decay_policy.hpp"

#include <gtest/gtest.h>

namespace
{

MapDecayPolicy policy(double decay_sec, int min_obs = 1)
{
  MapDecayPolicy p;
  p.decay_sec = decay_sec;
  p.min_obs = min_obs;
  return p;
}

} // namespace

// --- eviction --------------------------------------------------------------

TEST(MapDecayPolicy, EvictsOnlyPastTheThreshold)
{
  const MapDecayPolicy p = policy(300.0);
  EXPECT_FALSE(shouldEvictVoxel(1000.0, 701.0, false, p)); // 299 s old
  EXPECT_FALSE(shouldEvictVoxel(1000.0, 700.0, false, p)); // exactly 300 s
  EXPECT_TRUE(shouldEvictVoxel(1000.0, 699.0, false, p));  // 301 s old
}

TEST(MapDecayPolicy, DisabledByNonPositiveThreshold)
{
  // 0 and negative both mean "never evict" - what offline reprocessing of a
  // bag wants, and the escape hatch if decay is ever suspected of harm.
  EXPECT_FALSE(shouldEvictVoxel(1e6, 0.0, false, policy(0.0)));
  EXPECT_FALSE(shouldEvictVoxel(1e6, 0.0, false, policy(-1.0)));
}

TEST(MapDecayPolicy, PriorMapVoxelsNeverDecay)
{
  // A loaded session or a loop-closure map rebuild carries no live
  // observation time; ageing it out would delete the map we just loaded.
  EXPECT_TRUE(shouldEvictVoxel(1e6, 1.0, false, policy(300.0)));
  EXPECT_FALSE(shouldEvictVoxel(1e6, 1.0, true, policy(300.0)));
}

TEST(MapDecayPolicy, NeverObservedVoxelIsNotEvicted)
{
  // last_seen_t < 0 is a voxel created this instant that has not reached its
  // first observe() yet. Treating the sentinel as an age would delete it
  // before it was ever used.
  EXPECT_FALSE(shouldEvictVoxel(1000.0, -1.0, false, policy(300.0)));
}

TEST(MapDecayPolicy, ClockGoingBackwardsDoesNotFlushTheMap)
{
  // A looping bag, or a reset that rewinds the scan clock, makes `now` far
  // smaller than the stamps already in the map. That must read as "seen
  // recently", not as an enormous age - otherwise one sweep deletes
  // everything and the LIO has no map to match against.
  EXPECT_FALSE(shouldEvictVoxel(10.0, 5000.0, false, policy(300.0)));
}

TEST(MapDecayPolicy, StationaryVehicleStillAges)
{
  // The regression this whole change exists for: the previous rule keyed on
  // travelled distance, so a parked robot watching moving canopy evicted
  // nothing while the map kept growing. Scan time advances regardless.
  const MapDecayPolicy p = policy(300.0);
  const double stamped_while_parked = 100.0;
  EXPECT_TRUE(shouldEvictVoxel(stamped_while_parked + 301.0,
                               stamped_while_parked, false, p));
}

// --- match gate ------------------------------------------------------------

TEST(MapDecayPolicy, MinObsBelowTwoMatchesEverything)
{
  // The default. Preserves the pre-existing behaviour exactly, so enabling
  // decay does not silently also change what the registration matches.
  EXPECT_TRUE(voxelMatchable(0, false, policy(300.0, 1)));
  EXPECT_TRUE(voxelMatchable(0, false, policy(300.0, 0)));
  EXPECT_TRUE(voxelMatchable(0, false, policy(300.0, -5)));
}

TEST(MapDecayPolicy, UnconfirmedVoxelIsNotMatchedAgainst)
{
  const MapDecayPolicy p = policy(300.0, 3);
  EXPECT_FALSE(voxelMatchable(0, false, p));
  EXPECT_FALSE(voxelMatchable(2, false, p));
  EXPECT_TRUE(voxelMatchable(3, false, p));
  EXPECT_TRUE(voxelMatchable(4, false, p));
}

TEST(MapDecayPolicy, PriorMapVoxelsAreMatchableImmediately)
{
  // Prior-map voxels have never been "observed" by a live scan, so they would
  // otherwise sit unusable at obs_count 0 forever.
  EXPECT_TRUE(voxelMatchable(0, true, policy(300.0, 5)));
}

TEST(MapDecayPolicy, TransientIsKeptButNotConstraining)
{
  // A single-scan return - a person walking through, a gust in the canopy -
  // must not become a pose constraint, but must not be thrown away either:
  // if it turns out to be static it gets promoted on the next observation.
  const MapDecayPolicy p = policy(300.0, 2);
  EXPECT_FALSE(voxelMatchable(1, false, p));
  EXPECT_FALSE(shouldEvictVoxel(100.0, 99.0, false, p));
  EXPECT_TRUE(voxelMatchable(2, false, p));
}
