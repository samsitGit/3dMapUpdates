


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
    TraceProcessor traceProcessor(basemap, trace);
    basemap->setSampleSize(0.5);
    std::cout << "Starting trace processor" << std::endl;
    traceProcessor.start();
    return 0;
}

void sanityTest() {
    NearestVector nearestVector;
    PointCloudPtr cloud = trace->loadFrame(0);
    nearestVector.addVectors(getPointCloudVectors(cloud));
    for (int i = 0; i < cloud->size(); i++) {
        auto& point = cloud->points[i];
        auto closest_points = nearestVector.getSimilarVectors(Eigen::Vector3f(point.x, point.y, point.z), 1);
        auto& closest_point = closest_points[0];
        if (closest_point != Eigen::Vector3f(point.x, point.y, point.z)) {
            throw std::runtime_error("Error: Closest point is not the same as the point itself");
        }
    }
}

void testSimilarity() {
    NearestVector nearestVector;
	std::vector<Eigen::Vector3f> vectors;
    for (int i = 0; i < 100; i++) {
        auto noise=Eigen::Vector3f::Random();
		vectors.push_back(Eigen::Vector3f(i, i, i)+noise);
	}
	nearestVector.addVectors(vectors);
	auto similar_vectors = nearestVector.getSimilarVectors(vectors[50], 10);
    for (auto& vec : similar_vectors) {
		//print angle between vectors
        std::cout << angleBetweenVectors(vectors[50], vec) << std::endl;
	
	}   
}

double angleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
	return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}


void clipMapByRadius(const PointCloudPtr& target_cloud, float radius) {
	PointCloudPtr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
    PointCloudPtr transformed_frame(new pcl::PointCloud<pcl::PointXYZ>);
    if (downSample) {
		downSamplePointCloud(basemap_pcd, basemap_pcd);
	}
    pcl::transformPointCloud(*frame, *transformed_frame, pose);
    *merged = *basemap_pcd + *transformed_frame;
    
}


void testClipping() {
    auto basemap_pcd = basemap->getMap();
    std::string dir = "C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/localized_data/";
    auto frame1 = trace->loadFrame(1);
    Eigen::Matrix4f pose = trace->poses[0];
    //Transform target cloud so the origin is at the pose of the first frame


    //clipMapByRadius(basemap_pcd, 200);
    //merge the clipped map with the first frame
    PointCloudPtr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    mergeFrameWithBaseMap(basemap_pcd, frame1,merged_cloud, pose,true);
    saveTransformedCloud(merged_cloud, "merged_cloud_unclipped.pcd");
}

void testNormalization() {
    auto basemap_pcd = basemap->getMap();
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

double alignAndCalculateError(int frame, const PointCloudPtr& basemap_pcd,std::string& dir) {
    auto input_cloud = trace->loadFrame(frame);
    Eigen::Matrix4f pose = frame==1?trace->poses[0]:prevTransformation;
	PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto ndt = performNDTAlignment(input_cloud, basemap_pcd, output_cloud, pose,trace->poses[frame-1]);
    std::cout << ndt.getFitnessScore() << std::endl;
    PointCloudPtr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto finalTransformation = ndt.getFinalTransformation();
    mergeFrameWithBaseMap(basemap_pcd, input_cloud, merged_cloud,finalTransformation , false);
    saveTransformedCloud(merged_cloud, dir + std::to_string(frame) + ".pcd");
    prevTransformation = finalTransformation;
	return euclideanDistance(pose.block<3, 1>(0, 3), finalTransformation.block<3, 1>(0, 3));
}

void clipPointCloud(const PointCloudPtr& target_cloud, const PointCloudPtr& input_cloud, const Eigen::Matrix4f& pose) {
	PointCloudPtr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::savePCDFileASCII("clipped_cloud.pcd", *clipped_cloud);
	std::cout << "Clipped cloud saved to clipped_cloud.pcd" << std::endl;
}

void downSamplePointCloud(const PointCloudPtr& input_cloud, const PointCloudPtr& filtered_cloud,float leaf_size) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    std ::cout<< "Raw cloud contains " << input_cloud->size() << " data points" << std::endl;
    approximate_voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size() << " data points" << std::endl;
}





void saveTransformedCloud(const PointCloudPtr& output_cloud, const std::string& file_path) {
    pcl::io::savePCDFileASCII(file_path, *output_cloud);
    std::cout << "Saved transformed cloud to " << file_path << std::endl;
}
