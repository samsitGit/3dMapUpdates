#pragma once
#include "utils.h"
#include <pcl/filters/crop_box.h>
#include <pcl/surface/gp3.h>
#include "Cluster.h"
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class Basemap {
	private:
		std::string directory;
		std::string mapPath;
		PointCloudPtr map,new_points;
		pcl::KdTreeFLANN<pcl::PointXYZ> original_map_tree,appeared_points_tree;
	public:
		Basemap(const std::string& directory);
		void downSample(float leaf_size=0.5);
		PointCloudPtr clipByRadius(Eigen::Matrix4f transformation, float radius);
		PointCloudPtr getMap();
		PointCloudPtr integrateFrame(const PointCloudPtr& frame, const Eigen::Matrix4f& pose);
		PointCloudPtr integrateClippedMap(const PointCloudPtr& map,const PointCloudPtr& frame, const Eigen::Matrix4f& pose);
		void addPcd(const PointCloudPtr& frame);
		std::vector<NeighborPoint> getNeighbors(const pcl::PointXYZ& point, int k);
		void addCluster(Cluster& cluster);
		std::vector<NeighborPoint> getMapOnlyNeighbors(const pcl::PointXYZ& point, int k);
		pcl::PointXYZ getPoint(int index);
		//convert base map to mesh and return it
		pcl::PolygonMesh getMesh();

};