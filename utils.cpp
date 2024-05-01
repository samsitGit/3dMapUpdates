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


PointCloudPtr loadPointCloud(const std::string& file_path) {
	PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

int existsFile(const std::string& file_path) {
	std::ifstream file(file_path);
	return file.good();
}

int existsDirectory(const std::string& directory) {
	struct stat info;
	if (stat(directory.c_str(), &info) != 0) {
		return 0;
	}
	else if (info.st_mode & S_IFDIR) {
		return 1;
	}
	else {
		return 0;
	}
}

double euclideanDistance(const Eigen::Matrix<float, 3, 1>& pose1, const Eigen::Matrix<float, 3, 1>& pose2) {
	return (pose1 - pose2).norm();
}

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> performNDTAlignment(const PointCloudPtr& input_cloud,
	const PointCloudPtr& target_cloud, PointCloudPtr& output_cloud, Eigen::Matrix4f& pose, Eigen::Matrix4f& ground_truth) {

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// Set the resolution (voxel size)
	float resolution = 0.5; // meters
	ndt.setResolution(resolution);

	// Set the step size
	float step_size = 2; // meters
	ndt.setStepSize(step_size);

	// Set the transformation epsilon
	float transformation_epsilon = 0.01f; // meters
	ndt.setTransformationEpsilon(transformation_epsilon);

	// Set the maximum number of iterations
	int max_iterations = 100;
	ndt.setMaximumIterations(max_iterations);

	ndt.setInputSource(input_cloud);
	ndt.setInputTarget(target_cloud);
	// create initial guess as copy of pose with noise 
	Eigen::Matrix4f init_guess = pose;
	//applyGPSSimulatedNoise(init_guess);
	auto translation = init_guess.block<3, 1>(0, 3);
	std::cout << "Translational error before alignment: " << euclideanDistance(ground_truth.block<3, 1>(0, 3), translation) << std::endl;
	ndt.align(*output_cloud, init_guess);
	std::cout << "Translational error after alignment: " << euclideanDistance(ground_truth.block<3, 1>(0, 3), ndt.getFinalTransformation().block<3, 1>(0, 3)) << std::endl;
	std::cout << ndt.getFinalNumIteration() << " iterations" << std::endl;
	return ndt;
}

void applyNoise(Eigen::Matrix4f& pose,float intensity) {
	Eigen::Vector3f noise = Eigen::Vector3f::Random() * intensity;
	pose(0, 3) += noise(0);
	pose(1, 3) += noise(1);
	pose(2, 3) += noise(2);
}

void downSampleCloud(float leaf_size, PointCloudPtr cloud) {
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*cloud);
}

void removeOutliers(PointCloudPtr cloud, int n_neighbors, float stddev) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(n_neighbors);
	sor.setStddevMulThresh(stddev);
	sor.filter(*cloud);
}


std::vector<Eigen::Vector3f> getPointCloudVectors(PointCloudPtr cloud) {
	std::vector<Eigen::Vector3f> vectors;
	for (int i = 0; i < cloud->size(); i++) {
		pcl::PointXYZ point = cloud->points[i];
		Eigen::Vector3f vec(point.x, point.y, point.z);
		vectors.push_back(vec);
	}
	return vectors;
}