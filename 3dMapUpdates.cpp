

#include <thread>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include "3dMapUpdates.h"
#include "Trace.h"
// TODO: Reference additional headers your program requires here.

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using ApproximateVoxelGrid = pcl::ApproximateVoxelGrid<pcl::PointXYZ>;
using NormalDistributionsTransform = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
// Function prototypes



using namespace std::chrono_literals;



int main() {
    // Loading point clouds
    auto trace = new Trace("C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/map_compression/data_1/ego/town02/w_dropoff/w_noise/vehicles=6/day1");
    std::string targetFilePath = "C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/map_compression/data_1/basemaps/town02/w_dropoff/w_noise/complete.pcd";
    auto target_cloud = loadPointCloud(targetFilePath);
    auto input_cloud = trace->loadFrame(1);

    // Filtering input scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    clipPointCloud(target_cloud, trace->poses);
    filterPointCloud(input_cloud, filtered_cloud);
    return 0;
    // Performing NDT alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto transformation = performNDTAlignment(filtered_cloud, target_cloud, output_cloud);

    // Saving transformed input cloud
    saveTransformedCloud(output_cloud, "room_scan2_transformed.pcd");

    return 0;
}

void clipPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const std::vector<Eigen::Matrix4f>& poses) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pose : poses) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*target_cloud, *transformed_cloud, pose);
		*clipped_cloud += *transformed_cloud;
	}
	pcl::io::savePCDFileASCII("clipped_cloud.pcd", *clipped_cloud);
	std::cout << "Clipped cloud saved to clipped_cloud.pcd" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& file_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file \n");
        exit(-1);
    }
    std::cout << "Loaded " << cloud->size() << " data points from " << file_path << std::endl;
    return cloud;
}

void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size() << " data points" << std::endl;
}

Eigen::Matrix4f performNDTAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);

    ndt.setInputSource(filtered_cloud);
    ndt.setInputTarget(target_cloud);

    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;

    return ndt.getFinalTransformation();
}

void saveTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, const std::string& file_path) {
    pcl::io::savePCDFileASCII(file_path, *output_cloud);
    std::cout << "Saved transformed cloud to " << file_path << std::endl;
}
