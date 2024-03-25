// 3dMapUpdates.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& file_path);
void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);
Eigen::Matrix4f performNDTAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);
void saveTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, const std::string& file_path);
void clipPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const std::vector<Eigen::Matrix4f>& poses);