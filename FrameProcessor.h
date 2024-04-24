#include "Trace.h"
#include "Basemap.h"
#include<pcl/registration/icp.h>
//#include <ctpl.h>

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class FrameProcessor {
private:
	Basemap* basemap;
	Eigen::Matrix4f prevTransformation;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
public:
	Trace* trace;
	FrameProcessor(Basemap* basemap, Trace* trace);
	std::vector<PointCloudPtr> processFrame(int frame);
	PointCloudPtr subtract(PointCloudPtr frame);
};