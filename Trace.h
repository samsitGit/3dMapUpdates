#pragma once
#include<thread>
#include "utils.h"


using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class Trace
{
	private:
		std::string directory;
		std::string pcdDirectory;
		std::string outputDirectory;
	public:
		int num_frames;
		std::vector<Eigen::Matrix4f> poses;	
		Trace(const std::string& directory);
		PointCloudPtr loadFrame(int frame) const;
		void saveFrame(const PointCloudPtr& frame, int frame_number,const std::string& label="0") const;
};

