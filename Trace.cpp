#include "Trace.h"

using namespace std::chrono_literals;

Trace::Trace(const std::string& directory) {
	this->directory = directory;
	this->pcdDirectory=directory+"/pcds";
	this->poses = fetchPoses(directory + "/pose.txt");
	this->num_frames = poses.size();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Trace::loadFrame(int frame) const{
	std::string file_path = pcdDirectory + "/" + std::to_string(frame) + ".pcd";
	// check if file exists
	if (!exists(file_path)) {
		std::cerr << "File " << file_path << " does not exist" << std::endl;
		return nullptr;
	}
	return loadPointCloud(file_path);
}

