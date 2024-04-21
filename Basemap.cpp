#include "Basemap.h"

Basemap::Basemap(const std::string& directory) {
	this->directory = directory;
	this->poses = fetchPoses(directory + "/pose.txt");
	this->mapPath = directory + "/complete.pcd";
	
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Basemap::loadMap() const {
	return loadPointCloud(mapPath);
}

