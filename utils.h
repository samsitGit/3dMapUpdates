#include<vector>
#include<string>
#include<fstream>
#include<iostream>
#include<Eigen/Core>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

std::vector<std::string> split(std::string s, std::string delimiter);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll);
PointCloud::Ptr loadPointCloud(const std::string& file_path);
std::vector<Eigen::Matrix4f> fetchPoses(const std::string& file_path);
int exists(const std::string& file_path);