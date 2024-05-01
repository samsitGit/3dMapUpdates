#pragma once
#include "utils.h"
#include<pcl/common/distances.h>


using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class Cluster {
	private:
		
		pcl::PointXYZ& centroid;
		int frame;
	
	public:
		PointCloudPtr cloud;
		Cluster(PointCloudPtr cloud,int frame);
		PointCloudPtr getCloud() { return cloud; }
		pcl::PointXYZ getCentroid() { return centroid; }
		int getFrame() { return frame; }
		double getDistance(Cluster& cluster) const;
		void merge(Cluster& cluster);
		int size() { return cloud->size(); }
		void print();
};

class ClusterTrace {
private:
	pcl::PointXYZ centroidSum;
	int lastFrameSeen=-1;
	int size;
	
	public:
		std::vector<Cluster> clusters;
		ClusterTrace();
		void addCluster(Cluster& cluster);
		double getDistance(ClusterTrace& clusterTrace);
		double getDistance(Cluster& cluster);
		pcl::PointXYZ getCentroid();
		PointCloudPtr getMergedCloud();
		int getSize() { return size; }
		int lastSeen() { return lastFrameSeen; }
		double getDistanceMoved();
		void print();
		void reset();
};


class ClusterProcessor {
	//Tracks clusters over time and merges them into cluster traces
	//These traces are used to determine if the object is stationary or moving
	private:
		std::vector<ClusterTrace> clusterTraces;
		double clusterDistanceThreshold,dynamicObjectThreshold, aggregateThreshold;
		int obsoleteThreshold = 100;
	public:
		ClusterProcessor(double clusterDistanceThreshold,double dynamicObjectThreshold,int aggregateThreshold);
		void addCluster(Cluster& cluster);
		std::vector<Cluster> getClusterUpdates(int currentFrame);
		std::vector<ClusterTrace> getClusterTraces() { return clusterTraces; }
		void setObsoleteThreshold(int threshold) { obsoleteThreshold = threshold; }
		void clearOldTraces(int frame);
};

