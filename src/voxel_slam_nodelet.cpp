// Minimal nodelet wrapper for Voxel-SLAM to avoid multiple definitions
#include "voxel_slam_nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <thread>

// Bring in Voxel-SLAM main API
extern "C" void voxel_slam_start(ros::NodeHandle &n);

namespace voxel_slam {

// We only spawn the same threads as the original main():
// - loop closure thread
// - global mapping thread
// - call odometry/local mapping in the current thread
// All publishers/subscribers are set up inside VOXEL_SLAM ctor.

namespace {
std::shared_ptr<std::thread> th_loop;
std::shared_ptr<std::thread> th_gba;
}

void VoxelSlamNodelet::onInit() {
	nh_ = getNodeHandle();
	pnh_ = getPrivateNodeHandle();

	NODELET_INFO("voxel_slam nodelet initializing...");

		// Start Voxel-SLAM core (spawns its own threads and publishers)
		voxel_slam_start(pnh_);

	NODELET_INFO("voxel_slam nodelet initialized");
}

} // namespace voxel_slam

PLUGINLIB_EXPORT_CLASS(voxel_slam::VoxelSlamNodelet, nodelet::Nodelet)

