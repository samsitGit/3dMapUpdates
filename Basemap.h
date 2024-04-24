#pragma once
#include "utils.h"
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class Basemap {
	private:
		std::string directory;
		std::string mapPath;
		PointCloudPtr map;
	public:
		Basemap(const std::string& directory);
		void downSample(float leaf_size=0.5);
		PointCloudPtr clipByRadius(Eigen::Matrix4f transformation, float radius);
		PointCloudPtr getMap() { return map; }
		PointCloudPtr integrateFrame(const PointCloudPtr& frame, const Eigen::Matrix4f& pose);
		PointCloudPtr integrateClippedMap(const PointCloudPtr& map,const PointCloudPtr& frame, const Eigen::Matrix4f& pose);
		
};