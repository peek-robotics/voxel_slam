// Minimal nodelet wrapper for Voxel-SLAM to avoid multiple definitions
#include "voxel_slam_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

#include <cstdio>
#include <cstdlib>

// Bring in Voxel-SLAM main API
extern "C" void voxel_slam_start(ros::NodeHandle &n);

namespace voxel_slam {

// voxel_slam_start() spawns the same threads as the original main() (loop
// closure, global mapping, odometry/local mapping), all detached. All
// publishers/subscribers are set up inside the VOXEL_SLAM ctor.

void VoxelSlamNodelet::onInit() {
	nh_ = getNodeHandle();
	pnh_ = getPrivateNodeHandle();

	NODELET_INFO("voxel_slam nodelet initializing...");

		// Start Voxel-SLAM core (spawns its own threads and publishers)
		voxel_slam_start(pnh_);

	NODELET_INFO("voxel_slam nodelet initialized");
}

// The threads voxel_slam_start() detaches run on pnh_ and on process-wide
// globals, and nothing can stop them. Unloaded from a manager that lives on,
// they would keep running on a dead NodeHandle while the respawned loader
// starts a second instance beside them. So take the whole manager down
// instead: it respawns, and every nodelet reloads into a clean process. When
// the manager itself is shutting down, the threads die with it as before.
VoxelSlamNodelet::~VoxelSlamNodelet() {
	if (!ros::ok() || ros::isShuttingDown())
		return;

	NODELET_FATAL("voxel_slam cannot be unloaded from a running manager; "
	              "exiting so the manager respawns");
	std::fflush(nullptr);
	std::_Exit(EXIT_FAILURE);
}

} // namespace voxel_slam

PLUGINLIB_EXPORT_CLASS(voxel_slam::VoxelSlamNodelet, nodelet::Nodelet)
