// Minimal nodelet wrapper for Voxel-SLAM
#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace voxel_slam {

class VoxelSlamNodelet final : public nodelet::Nodelet {
public:
	VoxelSlamNodelet() = default;
	~VoxelSlamNodelet() override = default;

private:
	void onInit() override;

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
};

} // namespace voxel_slam

