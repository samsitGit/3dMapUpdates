#include "Basemap.h"

Basemap::Basemap(const std::string& directory) {
	this->directory = directory;
	this->mapPath = directory + "/complete.pcd";
	//this->mapPath = directory + "/updated.pcd"; //Map updated with newly appeared points
	this->map = loadPointCloud(mapPath);
	this->new_points = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	original_map_tree.setInputCloud(map);
	this->leaf_size = 0.01;
}

PointCloudPtr Basemap::getMap() {
	PointCloudPtr complete_map(new pcl::PointCloud<pcl::PointXYZ>);
	*complete_map = *map + *new_points;
	return complete_map;
}


void Basemap::setSampleSize(float leaf_size) {
	downSampleCloud(leaf_size, map);
	downSampleCloud(leaf_size, new_points);
	this->leaf_size=leaf_size;
}

PointCloudPtr Basemap::clipByRadius(Eigen::Matrix4f transformation, float radius) {
	PointCloudPtr clipped_map(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> cropFilter;
	PointCloudPtr complete_map = getMap();
	cropFilter.setInputCloud(complete_map);
	cropFilter.setTranslation(transformation.block<3, 1>(0, 3));
	cropFilter.setMin(Eigen::Vector4f(-radius, -radius, -radius, 1.0));
	cropFilter.setMax(Eigen::Vector4f(radius, radius, radius, 1.0));
	cropFilter.filter(*clipped_map);
	return clipped_map;
}

PointCloudPtr Basemap::integrateFrame(const PointCloudPtr& frame, const Eigen::Matrix4f& pose) {
	PointCloudPtr transformed_frame(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr complete_map= getMap();
	pcl::transformPointCloud(*frame, *transformed_frame, pose);
	*complete_map = *map + *transformed_frame;
	return complete_map;
}

PointCloudPtr Basemap::integrateClippedMap(const PointCloudPtr& clipped_map, const PointCloudPtr& frame, const Eigen::Matrix4f& pose) {
	PointCloudPtr transformed_frame(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr complete_map=getMap();
	pcl::transformPointCloud(*frame, *transformed_frame, pose);
	*complete_map = *clipped_map + *transformed_frame;
	return complete_map;
}

void Basemap::addPcd(const PointCloudPtr& frame) { //TODO: update the tree
	*map += *frame;
}

std::vector<NeighborPoint> Basemap::getNeighbors(const pcl::PointXYZ& point, int k) {
	std::vector<int> pointIdxMap(k),pointIdxNew(k);
	std::vector<float> pointDistanceMap(k),pointDistanceNew(k);
	std::vector<NeighborPoint> neighborsMap,neighborsNew;
	if (original_map_tree.nearestKSearch(point, k, pointIdxMap, pointDistanceMap) > 0) {
		for (int i = 0; i < pointIdxMap.size(); i++) {
			NeighborPoint neighbor(pointIdxMap[i], pointDistanceMap[i]);
			neighborsMap.push_back(neighbor);
		}
	}

	if (appeared_points_tree.nearestKSearch(point, k, pointIdxNew, pointDistanceNew) > 0) {
		for (int i = 0; i < pointIdxNew.size(); i++) {
			NeighborPoint neighbor(pointIdxNew[i], pointDistanceNew[i]);
			neighborsNew.push_back(neighbor);
		}
	}

	//Find the k nearest neighbors from the map and the new points by comparing the distances
	std::vector<NeighborPoint> neighbors;
	int i = 0, j = 0;
	while (neighbors.size() < k) {
		if (i < neighborsMap.size() && j < neighborsNew.size()) {
			if (neighborsMap[i].distance < neighborsNew[j].distance) {
				neighbors.push_back(neighborsMap[i]);
				i++;
			}
			else {
				neighbors.push_back(neighborsNew[j]);
				j++;
			}
		}
		else if (i < neighborsMap.size()) {
			neighbors.push_back(neighborsMap[i]);
			i++;
		}
		else if (j < neighborsNew.size()) {
			neighbors.push_back(neighborsNew[j]);
			j++;
		}
	}
	return neighbors;
}

pcl::PointXYZ Basemap::getPoint(int index) {
	return map->points[index];
}

std::vector<NeighborPoint> Basemap::getMapOnlyNeighbors(const pcl::PointXYZ& point, int k) {
	std::vector<int> pointIdxMap(k);
	std::vector<float> pointDistanceMap(k);
	std::vector<NeighborPoint> neighborsMap;
	if (original_map_tree.nearestKSearch(point, k, pointIdxMap, pointDistanceMap) > 0) {
		for (int i = 0; i < pointIdxMap.size(); i++) {
			NeighborPoint neighbor(pointIdxMap[i], pointDistanceMap[i]);
			neighborsMap.push_back(neighbor);
		}
	}
	return neighborsMap;
}

void Basemap::addCluster(Cluster& cluster) {
	for (auto& point : *cluster.getCloud()) {
		new_points->push_back(point);
	}
	downSampleCloud(leaf_size, new_points);
	appeared_points_tree.setInputCloud(new_points);
}

void Basemap::saveMap(const std::string& filename) {
	auto map = getMap();
	std::string path= directory + "/" + filename;
	pcl::io::savePCDFileASCII(path, *map);
}