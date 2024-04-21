#include "utils.h"

std::vector<std::string> split(std::string s, std::string delimiter) {
	size_t pos_start = 0, pos_end, delim_len = delimiter.length();
	std::string token;
	std::vector<std::string> res;

	while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
		token = s.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		res.push_back(token);
	}

	res.push_back(s.substr(pos_start));
	return res;
}

Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose)
{
	Eigen::Matrix4f rotation_matrix = convert_YPR_to_rotation_matrix(std::stof(pose[4]), std::stof(pose[5]), std::stof(pose[6]));
	rotation_matrix(0, 3) = std::stof(pose[1]);
	rotation_matrix(1, 3) = std::stof(pose[2]);
	rotation_matrix(2, 3) = std::stof(pose[3]);

	return rotation_matrix;
}


Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll)
{
	Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
	yaw = (M_PI * yaw) / 180.0;
	pitch = (M_PI * pitch) / 180.0;
	roll = (M_PI * roll) / 180.0;
	rotation_matrix(0, 0) = std::cos(yaw) * std::cos(pitch);
	rotation_matrix(1, 0) = sin(yaw) * std::cos(pitch);
	rotation_matrix(2, 0) = -sin(pitch);


	rotation_matrix(0, 1) = std::cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * std::cos(roll);
	rotation_matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + std::cos(yaw) * std::cos(roll);
	rotation_matrix(2, 1) = std::cos(pitch) * sin(roll);

	rotation_matrix(0, 2) = std::cos(yaw) * sin(pitch) * std::cos(roll) + sin(yaw) * sin(roll);
	rotation_matrix(1, 2) = sin(yaw) * sin(pitch) * std::cos(roll) - std::cos(yaw) * sin(roll);
	rotation_matrix(2, 2) = std::cos(pitch) * std::cos(roll);

	return rotation_matrix;

}


PointCloud::Ptr loadPointCloud(const std::string& file_path) {
	PointCloud::Ptr cloud(new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
		PCL_ERROR("Couldn't read file \n");
		exit(-1);
	}
	std::cout << "Loaded " << cloud->size() << " data points from " << file_path << std::endl;
	return cloud;
}

std::vector<Eigen::Matrix4f> fetchPoses(const std::string& file_path) {
	std::ifstream file(file_path);
	std::vector<Eigen::Matrix4f> poses;
	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line)) {
			std::vector<std::string> tokens = split(line, ", ");
			Eigen::Matrix4f transformation_matrix = get_transformation_matrix(tokens);
			poses.push_back(transformation_matrix);
		}
	}
	return poses;
}

int exists(const std::string& file_path) {
	std::ifstream file(file_path);
	return file.good();
}