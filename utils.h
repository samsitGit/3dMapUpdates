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

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

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
