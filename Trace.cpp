#include "Trace.h"

using namespace std::chrono_literals;

Trace::Trace(const std::string& directory,int num_f) {
	this->directory = directory;
	this->pcdDirectory=directory+"/pcds";
	this->outputDirectory=directory+"/output";
	this->poses = fetchPoses(directory + "/pose.txt");
	this->num_frames = num_f==-1?poses.size():num_f;
	init();
}

void Trace::init() {
	//parallely load all frames
	ctpl::thread_pool p(8);
	std::vector<std::future<PointCloudPtr>> futures;
	for (int i = 0; i < num_frames; i++) {
		futures.push_back(p.push([this, i](int id) { return loadFrame(i); }));
	}
	for (int i = 0; i < num_frames; i++) {
		PointCloudPtr frame = futures[i].get();
		if (frame != nullptr) {
			frames.push_back(frame);
		}
	}
	p.stop(true);
}

PointCloudPtr Trace::loadFrame(int frame) const{
	if (frame >= num_frames) {
		std::cerr << "Frame " << frame << " does not exist" << std::endl;
		return nullptr;
	}
	if (frames.size() > frame) {
		return frames[frame];
	}
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
