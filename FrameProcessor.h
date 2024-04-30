#include "Trace.h"
#include "Basemap.h"
#include<pcl/registration/icp.h>
#include<pcl/segmentation/extract_clusters.h>
#include "Cluster.h"


using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class FrameProcessor {
private:
	Basemap* basemap;
	ctpl::thread_pool *pool;
	PointCloudPtr aggregated_frames;
	int aggregated_frames_count;
public:
	ClusterProcessor *appearedPointsProcessor,*disappearedPointsProcessor;
	Trace* trace;
	FrameProcessor(Basemap* basemap, Trace* trace);
	std::vector<PointCloudPtr> processFrame(int frame);
	PointCloudPtr subtract(PointCloudPtr frame);
	std::vector<PointCloudPtr> extractClusters(PointCloudPtr frame);
	PointCloudPtr rayTraceDisappearedPoints(PointCloudPtr frame,Eigen::Matrix4f origin, float resolution=0.05);
	std::vector<NeighborPoint> rayTracePoint(int id,pcl::PointXYZ point,Eigen::Matrix4f origin, float resolution=0.05);
	PointCloudPtr subtractMap(PointCloudPtr frame, PointCloudPtr map);
};