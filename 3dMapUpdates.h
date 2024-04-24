// 3dMapUpdates.h : Include file for standard system include files,
// or project specific include files.


#include "Trace.h"
#include "Basemap.h"
#include "TraceProcessor.h"
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


void downSamplePointCloud(const PointCloudPtr& input_cloud, const PointCloudPtr& filtered_cloud,float leaf_size=0.5);
void saveTransformedCloud(const PointCloudPtr& output_cloud, const std::string& file_path);
void clipPointCloud(const PointCloudPtr& target_cloud, const PointCloudPtr& input_cloud, const Eigen::Matrix4f& pose);
void testNormalization();
double alignAndCalculateError(int frame, const PointCloudPtr& target_cloud, std::string& dir);
void mergeFrameWithBaseMap(const PointCloudPtr& basemap, const PointCloudPtr& frame, const PointCloudPtr& merged, const Eigen::Matrix4f& pose, bool downSample = false);
void clipMapByRadius(const PointCloudPtr& target_cloud, float radius);
void testClipping();
