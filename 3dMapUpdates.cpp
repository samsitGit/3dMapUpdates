


#include "3dMapUpdates.h"

// TODO: Reference additional headers your program requires here.
using ApproximateVoxelGrid = pcl::ApproximateVoxelGrid<pcl::PointXYZ>;
using NormalDistributionsTransform = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
using namespace std::chrono_literals;
// Loading point clouds
auto trace = new Trace("C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/map_compression/data_1/ego/town02/w_dropoff/w_noise/vehicles=6/day1");
auto basemap = new Basemap("C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/map_compression/data_1/basemaps/town02/w_dropoff/w_noise");
Eigen::Matrix4f prevTransformation = Eigen::Matrix4f::Identity();

int main() {


    testNormalization();

    return 0;
}

double euclideanDistance(const Eigen::Matrix<float, 3, 1>& pose1, const Eigen::Matrix<float, 3, 1>& pose2) {
	return (pose1 - pose2).norm();
}

void clipMapByRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, float radius) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::CropBox<pcl::PointXYZ> crop_box_filter;
	crop_box_filter.setMin(Eigen::Vector4f(-radius, -radius, -radius, 1.0));
	crop_box_filter.setMax(Eigen::Vector4f(radius, radius, radius, 1.0));
	crop_box_filter.setInputCloud(target_cloud);
	crop_box_filter.filter(*clipped_cloud);

	pcl::io::savePCDFileASCII("clipped_cloud_map.pcd", *clipped_cloud);
	std::cout << "Clipped cloud saved to clipped_cloud.pcd" << std::endl;
    *target_cloud = *clipped_cloud;
}

void mergeFrameWithBaseMap(const PointCloudPtr& basemap_pcd, const PointCloudPtr& frame, const PointCloudPtr& merged,const Eigen::Matrix4f& pose,bool downSample) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_frame(new pcl::PointCloud<pcl::PointXYZ>);
    if (downSample) {
		downSamplePointCloud(basemap_pcd, basemap_pcd);
	}
    pcl::transformPointCloud(*frame, *transformed_frame, pose);
    *merged = *basemap_pcd + *transformed_frame;
    
}


void testClipping() {
    auto basemap_pcd = basemap->loadMap();
    std::string dir = "C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/localized_data/";
    auto frame1 = trace->loadFrame(1);
    Eigen::Matrix4f pose = trace->poses[0];
    //Transform target cloud so the origin is at the pose of the first frame


    //clipMapByRadius(basemap_pcd, 200);
    //merge the clipped map with the first frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    mergeFrameWithBaseMap(basemap_pcd, frame1,merged_cloud, pose,true);
    saveTransformedCloud(merged_cloud, "merged_cloud_unclipped.pcd");
}

void testNormalization() {
    auto basemap_pcd = basemap->loadMap();
    downSamplePointCloud(basemap_pcd, basemap_pcd);
    std::string dir = "C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/localized_data/";
    std::vector<double> translation_errors;
    for (int frame_number = 1; frame_number <= trace->num_frames; frame_number++) {
        auto translational_error = alignAndCalculateError(frame_number, basemap_pcd,dir);
        translation_errors.push_back(translational_error);
        std::cout<<"Frame "<<frame_number<<": Translation error after alignment - "<<translational_error<<std::endl;
	}
    std::ofstream file(dir+"scores.txt");
    for (auto error : translation_errors) {
		file<<error<<std::endl;
    }
	file.close();
	std::cout<<"Scores saved to "<<dir+"scores.txt"<<std::endl;
}

double alignAndCalculateError(int frame, const pcl::PointCloud<pcl::PointXYZ>::Ptr& basemap_pcd,std::string& dir) {
    auto input_cloud = trace->loadFrame(frame);
    Eigen::Matrix4f pose = frame==1?trace->poses[0]:prevTransformation;
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto ndt = performNDTAlignment(input_cloud, basemap_pcd, output_cloud, pose,trace->poses[frame-1]);
    std::cout << ndt.getFitnessScore() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto finalTransformation = ndt.getFinalTransformation();
    mergeFrameWithBaseMap(basemap_pcd, input_cloud, merged_cloud,finalTransformation , false);
    saveTransformedCloud(merged_cloud, dir + std::to_string(frame) + ".pcd");
    prevTransformation = finalTransformation;
	return euclideanDistance(pose.block<3, 1>(0, 3), finalTransformation.block<3, 1>(0, 3));
}

void clipPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const Eigen::Matrix4f& pose) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::savePCDFileASCII("clipped_cloud.pcd", *clipped_cloud);
	std::cout << "Clipped cloud saved to clipped_cloud.pcd" << std::endl;
}

void downSamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,float leaf_size) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    std ::cout<< "Raw cloud contains " << input_cloud->size() << " data points" << std::endl;
    approximate_voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size() << " data points" << std::endl;
}

void applyGPSSimulatedNoise(Eigen::Matrix4f& pose) {
	Eigen::Vector3f noise = Eigen::Vector3f::Random() * 5;
	pose(0, 3) += noise(0);
	pose(1, 3) += noise(1);
	pose(2, 3) += noise(2);
}

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> performNDTAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, Eigen::Matrix4f& pose,Eigen::Matrix4f& ground_truth) {

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // Set the resolution (voxel size)
    float resolution = 0.5f; // meters
    ndt.setResolution(resolution);

    // Set the step size
    float step_size = 0.2f; // meters
    ndt.setStepSize(step_size);

    // Set the transformation epsilon
    float transformation_epsilon = 0.01f; // meters
    ndt.setTransformationEpsilon(transformation_epsilon);

    // Set the maximum number of iterations
    int max_iterations = 100;
    ndt.setMaximumIterations(max_iterations);

    // Set the outlier ratio
    float outlier_ratio = 0.1f;
    ndt.setOulierRatio(outlier_ratio);

    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);
    // create initial guess as copy of pose with noise with different references
    Eigen::Matrix4f init_guess = pose;
    //applyGPSSimulatedNoise(init_guess);
    auto translation = init_guess.block<3, 1>(0, 3);
    std::cout<<"Translational error before alignment: "<<euclideanDistance(ground_truth.block<3,1>(0,3), translation) << std::endl;
    ndt.align(*output_cloud, init_guess);
    std::cout<<"Translational error after alignment: "<<euclideanDistance(ground_truth.block<3, 1>(0, 3), ndt.getFinalTransformation().block<3, 1>(0, 3))<<std::endl;
    std::cout<<ndt.getFinalNumIteration()<<" iterations"<<std::endl;
    return ndt;
}

void saveTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, const std::string& file_path) {
    pcl::io::savePCDFileASCII(file_path, *output_cloud);
    std::cout << "Saved transformed cloud to " << file_path << std::endl;
}
