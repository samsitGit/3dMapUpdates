#include "utils.h"

class Basemap {
	private:
		std::string directory;
		std::vector<Eigen::Matrix4f> poses;
		std::string mapPath;
	
	public:
		Basemap(const std::string& directory);
		pcl::PointCloud<pcl::PointXYZ>::Ptr loadMap() const;
};