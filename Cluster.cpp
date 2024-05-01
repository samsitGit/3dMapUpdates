#include "Cluster.h"

double Cluster::getDistance(Cluster& cluster) const{
	return pcl::euclideanDistance(centroid, cluster.centroid);
}

void Cluster::merge(Cluster& cluster) {
	*cloud += *cluster.cloud;
}

Cluster::Cluster(PointCloudPtr cloud,int frame):centroid(cloud->points[0]) {
	this->cloud = cloud;
	this->frame = frame;
	centroid.x = 0;
	centroid.y = 0;
	centroid.z = 0;
	for (auto& point : *cloud) {
		centroid.x += point.x;
		centroid.y += point.y;
		centroid.z += point.z;
	}
	centroid.x /= cloud->size();
	centroid.y /= cloud->size();
	centroid.z /= cloud->size();
}



void Cluster::print() {
	std::cout << "Cluster: " << std::endl;
	std::cout << "Frame: " << frame << std::endl;
	std::cout << "Size: " << cloud->size() << std::endl;
	std::cout << "Centroid: " << centroid.x << " " << centroid.y << " " << centroid.z << std::endl;
}

ClusterTrace::ClusterTrace() {
	this->clusters = std::vector<Cluster>();
	this->centroidSum = pcl::PointXYZ();
	this->size = 0;
}

void ClusterTrace::addCluster(Cluster& cluster) {
	clusters.push_back(cluster);
	centroidSum.x += cluster.getCentroid().x;
	centroidSum.y += cluster.getCentroid().y;
	centroidSum.z += cluster.getCentroid().z;
	size ++;
	lastFrameSeen = cluster.getFrame();
}

pcl::PointXYZ ClusterTrace::getCentroid() {
	pcl::PointXYZ centroid;
	centroid.x = centroidSum.x / size;
	centroid.y = centroidSum.y / size;
	centroid.z = centroidSum.z / size;
	return centroid;
}

double ClusterTrace::getDistance(ClusterTrace& clusterTrace) {
	pcl::PointXYZ c1 = this->getCentroid();
	pcl::PointXYZ c2 = clusterTrace.getCentroid();

	return sqrt(pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2) + pow(c1.z - c2.z, 2));
}

double ClusterTrace::getDistance(Cluster& cluster) {
	pcl::PointXYZ c1 = this->getCentroid();
	pcl::PointXYZ c2 = cluster.getCentroid();

	return sqrt(pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2) + pow(c1.z - c2.z, 2));
}


PointCloudPtr ClusterTrace::getMergedCloud() {
	PointCloudPtr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (auto& cluster : clusters) {
		*mergedCloud += *cluster.getCloud();
	}
	removeOutliers(mergedCloud, 50, 1.0);
	downSampleCloud(0.01, mergedCloud);
	return mergedCloud;
}

double ClusterTrace::getDistanceMoved() {
	return clusters[0].getDistance(clusters[clusters.size() - 1]);
}

void ClusterTrace::print() {
	std::cout << "ClusterTrace: " << std::endl;
	std::cout<< "Size: " << size << std::endl;
	std::cout << "Centroid: " << getCentroid().x << " " << getCentroid().y << " " << getCentroid().z << std::endl;
	std::cout << "Last Frame Seen: " << lastFrameSeen << std::endl;
	std::cout<< "Distance Moved: " << getDistanceMoved() << std::endl;
}

void ClusterTrace::reset() {
	clusters.clear();
	centroidSum = pcl::PointXYZ();
	size = 0;
	lastFrameSeen = -1;
}


ClusterProcessor::ClusterProcessor(double clusterDistanceThreshold,double dynamicObjectThreshold,int aggregateThreshold) {
	this->clusterDistanceThreshold = clusterDistanceThreshold;
	this->dynamicObjectThreshold = dynamicObjectThreshold;
	this-> aggregateThreshold= aggregateThreshold;
}

void ClusterProcessor::addCluster(Cluster& cluster) {
	if (clusterTraces.size() == 0) {
		ClusterTrace clusterTrace;
		clusterTrace.addCluster(cluster);
		clusterTraces.push_back(clusterTrace);
	}
	else {
		double minDistance = std::numeric_limits<double>::max();
		int minIndex = -1;
		for (int i = 0; i < clusterTraces.size(); i++) {
			double distance = clusterTraces[i].getDistance(cluster);
			
			if (distance < minDistance) {
				minDistance = distance;
				minIndex = i;
			}
		}
		if (minDistance < clusterDistanceThreshold) {
			clusterTraces[minIndex].addCluster(cluster);
		}
		else {
			ClusterTrace clusterTrace;
			clusterTrace.addCluster(cluster);
			clusterTraces.push_back(clusterTrace);
		}
	}
}

std::vector<Cluster> ClusterProcessor::getClusterUpdates(int currentFrame) {
	std::vector<Cluster> clusterUpdates;
	clearOldTraces(currentFrame);
	for (int i = 0; i < clusterTraces.size(); i++) {
		ClusterTrace& clusterTrace = clusterTraces[i];
		if (clusterTrace.lastSeen() == -1) continue;
		if (clusterTrace.getDistanceMoved() > dynamicObjectThreshold) continue;
		if(clusterTrace.clusters.size() < aggregateThreshold) continue;
		//clusterTrace.print();
		auto mergedCluster = Cluster(clusterTrace.getMergedCloud(), currentFrame);
		clusterUpdates.push_back(mergedCluster);
		clusterTrace.reset();
		clusterTrace.addCluster(mergedCluster);
		//clusterTraces.erase(clusterTraces.begin() + i);
		
	}
	return clusterUpdates;
}

void ClusterProcessor::clearOldTraces(int frame) {
	for (int i = 0; i < clusterTraces.size(); i++) {
		if (clusterTraces[i].lastSeen() < 0) continue;
		if (frame - clusterTraces[i].lastSeen() > obsoleteThreshold) {
			clusterTraces.erase(clusterTraces.begin() + i);
		}

	}
}