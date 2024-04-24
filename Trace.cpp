#include "Trace.h"

using namespace std::chrono_literals;

Trace::Trace(const std::string& directory) {
	this->directory = directory;
	this->pcdDirectory=directory+"/pcds";
	this->outputDirectory=directory+"/output";
	this->poses = fetchPoses(directory + "/pose.txt");
	this->num_frames = poses.size();

}

PointCloudPtr Trace::loadFrame(int frame) const{
	std::string file_path = pcdDirectory + "/" + std::to_string(frame+1) + ".pcd";
	// check if file exists
	if (!existsFile(file_path)) {
		std::cerr << "File " << file_path << " does not exist" << std::endl;
		return nullptr;
	}
	return loadPointCloud(file_path);
}

void Trace::saveFrame(const PointCloudPtr& frame,int frame_number, const std::string& label) const {
	// check if output directory exists
	if (!existsDirectory(outputDirectory)) {
		std::cerr << "Output directory " << outputDirectory << " does not exist" << std::endl;
		return;
	}
	std::string dir = outputDirectory + "/" + label;
	// check if label directory exists
	if (!existsDirectory(dir)) {
		std::cerr << "Label directory " << dir << " does not exist" << std::endl;
		return;
	}

	std::string file_path = dir + "/" + std::to_string(frame_number) + ".pcd";
	pcl::io::savePCDFileASCII(file_path, *frame);
}
