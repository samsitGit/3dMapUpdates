#include "Trace.h"
#include "Basemap.h"
#include<pcl/registration/icp.h>
#include<pcl/segmentation/extract_clusters.h>
#include "Cluster.h"
#include "hnswlib.h"
#include "Timer.h"

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class NearestVector {

private:
	PointCloudPtr cloud;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	std::vector<Eigen::Vector3f> vectors;
public:
	NearestVector();
	void addVectors(std::vector<Eigen::Vector3f> vectors);
	std::vector<Eigen::Vector3f> getSimilarVectors(Eigen::Vector3f vector, int k);
};


class FrameProcessor {
private:
	ctpl::thread_pool *pool;
	PointCloudPtr aggregated_frames;
	int aggregated_frames_count;
public:
	Basemap* basemap;
	Timer timer;
	ClusterProcessor *appearedPointsProcessor,*disappearedPointsProcessor;
	Trace* trace;
	FrameProcessor(Basemap* basemap, Trace* trace);
	std::vector<PointCloudPtr> processFrame(int frame);
	PointCloudPtr subtract(PointCloudPtr frame);
	std::vector<Eigen::Vector3f> vectorRayTracePoint(Eigen::Vector3f point, NearestVector* nearestVector, float radius);
	PointCloudPtr vectorRayTraceDisappearedPoints(PointCloudPtr frame, PointCloudPtr map, Eigen::Matrix4f origin, float radius);
	std::vector<PointCloudPtr> extractClusters(PointCloudPtr frame);
	PointCloudPtr rayTraceDisappearedPoints(PointCloudPtr frame,Eigen::Matrix4f origin, float resolution=0.05);
	std::vector<NeighborPoint> rayTracePoint(int id,pcl::PointXYZ point,Eigen::Matrix4f origin, float resolution=0.05);
	PointCloudPtr subtractMap(PointCloudPtr frame, PointCloudPtr map);

};
