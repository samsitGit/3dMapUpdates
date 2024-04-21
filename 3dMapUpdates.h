// 3dMapUpdates.h : Include file for standard system include files,
// or project specific include files.

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "Trace.h"
#include "Basemap.h"
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


void downSamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,float leaf_size=0.5);
pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> performNDTAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, Eigen::Matrix4f& pose, Eigen::Matrix4f& ground_truth);
void saveTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, const std::string& file_path);
void clipPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const Eigen::Matrix4f& pose);
void applyGPSSimulatedNoise(Eigen::Matrix4f& pose);
void testNormalization();
double alignAndCalculateError(int frame, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, std::string& dir);
double euclideanDistance(const Eigen::Matrix<float, 3, 1>& pose1, const Eigen::Matrix<float, 3, 1>& pose2);
void mergeFrameWithBaseMap(const PointCloudPtr& basemap, const PointCloudPtr& frame, const PointCloudPtr& merged, const Eigen::Matrix4f& pose, bool downSample = false);
void clipMapByRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, float radius);
void testClipping();
