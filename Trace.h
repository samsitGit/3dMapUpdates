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
		std::vector<PointCloudPtr> frames;
	public:
		int num_frames;
		void init();
		std::vector<Eigen::Matrix4f> poses;	
		Trace(const std::string& directory,int num_f=-1);
		PointCloudPtr loadFrame(int frame) const;
		void saveFrame(const PointCloudPtr& frame, int frame_number,const std::string& label="0") const;
};

