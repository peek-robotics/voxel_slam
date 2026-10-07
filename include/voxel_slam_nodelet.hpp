// Minimal nodelet wrapper for Voxel-SLAM
#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace voxel_slam {

class VoxelSlamNodelet final : public nodelet::Nodelet {
public:
	VoxelSlamNodelet() = default;
	~VoxelSlamNodelet() override;

private:
	void onInit() override;

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	// True once voxel_slam_start() returned: only then do detached threads exist.
	bool started_ = false;
};

} // namespace voxel_slam

