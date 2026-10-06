#include "map_decay_policy.hpp"

#include <gtest/gtest.h>

#include <map>
#include <set>
#include <vector>

namespace
{

MapDecayPolicy policy(double decay_sec)
{
  MapDecayPolicy p;
  p.decay_sec = decay_sec;
  return p;
}

struct FakeVoxel
{
  double last_seen_t;
};

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

// --- sweep -----------------------------------------------------------------

namespace
{

// A map shaped like surf_map: key -> owning pointer, with the sweep handing
// evicted pointers to `retire` instead of freeing them.
struct SweepFixture
{
  std::vector<FakeVoxel> storage;
  std::map<int, FakeVoxel*> map;
  std::set<int> in_window;
  std::vector<FakeVoxel*> retired;

  explicit SweepFixture(const std::vector<double>& stamps) : storage(stamps.size())
  {
    for (size_t i = 0; i < stamps.size(); ++i)
    {
      storage[i].last_seen_t = stamps[i];
      map[static_cast<int>(i)] = &storage[i];
    }
  }

  int sweep(double now, const MapDecayPolicy& p)
  {
    return sweepDecayedVoxels(
            map, now, p, [this](int k) { return in_window.count(k) != 0; },
            [this](FakeVoxel* v) { retired.push_back(v); });
  }
};

} // namespace

TEST(MapDecayPolicy, SweepRemovesOnlyStaleVoxelsAndRetiresEachOnce)
{
  SweepFixture f({10.0, 500.0, 50.0, 790.0, -1.0});
  EXPECT_EQ(f.sweep(800.0, policy(300.0)), 2);

  EXPECT_EQ(f.map.size(), 3u);
  EXPECT_EQ(f.map.count(0), 0u);
  EXPECT_EQ(f.map.count(2), 0u);
  ASSERT_EQ(f.retired.size(), 2u);
  EXPECT_EQ(f.retired[0], &f.storage[0]);
  EXPECT_EQ(f.retired[1], &f.storage[2]);
}

TEST(MapDecayPolicy, SweepSparesVoxelsTheSlidingWindowStillHolds)
{
  // Freeing a voxel the local BA still references is a use-after-free, so the
  // window's veto outranks age.
  SweepFixture f({10.0, 20.0});
  f.in_window.insert(0);
  EXPECT_EQ(f.sweep(800.0, policy(300.0)), 1);
  EXPECT_EQ(f.map.count(0), 1u);
  EXPECT_EQ(f.map.count(1), 0u);
}

TEST(MapDecayPolicy, SweepIsANoOpWhenDisabledOrBeforeTheFirstScan)
{
  SweepFixture f({10.0, 20.0});
  EXPECT_EQ(f.sweep(800.0, policy(0.0)), 0);
  // now <= 0 is "no scan time yet"; nothing can be judged old against it.
  EXPECT_EQ(f.sweep(0.0, policy(300.0)), 0);
  EXPECT_EQ(f.map.size(), 2u);
  EXPECT_TRUE(f.retired.empty());
}

TEST(MapDecayPolicy, SweepEmptiesAMapThatIsEntirelyStale)
{
  // Erase-while-iterating over every element, including the last.
  SweepFixture f({1.0, 2.0, 3.0, 4.0});
  EXPECT_EQ(f.sweep(1000.0, policy(300.0)), 4);
  EXPECT_TRUE(f.map.empty());
}
