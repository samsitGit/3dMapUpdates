#pragma once
#include<vector>
#include<string>
#include<fstream>
#include<iostream>
#include<Eigen/Core>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include<pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "ctpl.h"

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class NeighborPoint {
	public:
	int index;
	float distance;
	
	NeighborPoint(int index, float distance) : index(index), distance(distance) {}


};

std::vector<std::string> split(std::string s, std::string delimiter);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll);
PointCloudPtr loadPointCloud(const std::string& file_path);
std::vector<Eigen::Matrix4f> fetchPoses(const std::string& file_path);
int existsFile(const std::string& file_path);
int existsDirectory(const std::string& directory);
pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> performNDTAlignment(const PointCloudPtr& input_cloud, const PointCloudPtr& target_cloud, PointCloudPtr& output_cloud, Eigen::Matrix4f& pose, Eigen::Matrix4f& ground_truth);
double euclideanDistance(const Eigen::Matrix<float, 3, 1>& pose1, const Eigen::Matrix<float, 3, 1>& pose2);
void applyNoise(Eigen::Matrix4f& pose,float intensity);
void downSampleCloud(float leaf_size, PointCloudPtr cloud);
void removeOutliers(PointCloudPtr cloud, int n_neighbors, float stddev);
std::vector<Eigen::Vector3f> getPointCloudVectors(PointCloudPtr cloud);