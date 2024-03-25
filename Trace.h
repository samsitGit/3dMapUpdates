#pragma once
#include<string>
#include "3dMapUpdates.h"

class Trace
{
	std::string directory;
	std::string pcdDirectory;
	public:
	std::vector<Eigen::Matrix4f> poses;
	Trace(const std::string& directory);
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadFrame(int frame) const;
	std::vector<Eigen::Matrix4f> fetchPoses(const std::string& file_path) const;
};

std::vector<std::string> split(std::string s, std::string delimiter);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll);
