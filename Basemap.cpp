#include "Basemap.h"

Basemap::Basemap(const std::string& directory) {
	this->directory = directory;
	this->mapPath = directory + "/complete.pcd";
	this->map = loadPointCloud(mapPath);
}

void Basemap::downSample(float leaf_size) {
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(map);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*map);
}

PointCloudPtr Basemap::clipByRadius(Eigen::Matrix4f transformation, float radius) {
	PointCloudPtr clipped_map(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> cropFilter;
	cropFilter.setInputCloud(map);
	cropFilter.setTranslation(transformation.block<3, 1>(0, 3));
	cropFilter.setMin(Eigen::Vector4f(-radius, -radius, -radius, 1.0));
	cropFilter.setMax(Eigen::Vector4f(radius, radius, radius, 1.0));
	cropFilter.filter(*clipped_map);
	return clipped_map;
}

PointCloudPtr Basemap::integrateFrame(const PointCloudPtr& frame, const Eigen::Matrix4f& pose) {
	PointCloudPtr transformed_frame(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr new_map(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*frame, *transformed_frame, pose);
	*new_map = *map + *transformed_frame;
	return new_map;
}

PointCloudPtr Basemap::integrateClippedMap(const PointCloudPtr& clipped_map, const PointCloudPtr& frame, const Eigen::Matrix4f& pose) {
	PointCloudPtr transformed_frame(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr new_map(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*frame, *transformed_frame, pose);
	*new_map = *clipped_map + *transformed_frame;
	return new_map;
}