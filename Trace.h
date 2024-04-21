
#include<thread>
#include "utils.h"


using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class Trace
{
	private:
		std::string directory;
		std::string pcdDirectory;
	public:
		int num_frames;
		std::vector<Eigen::Matrix4f> poses;	
		Trace(const std::string& directory);
		pcl::PointCloud<pcl::PointXYZ>::Ptr loadFrame(int frame) const;
	
};

