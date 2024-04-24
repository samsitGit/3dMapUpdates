#include "FrameProcessor.h"

FrameProcessor::FrameProcessor(Basemap* basemap, Trace* trace) : basemap(basemap), trace(trace)
{
	this->prevTransformation = trace->poses[0];
	kdtree.setInputCloud(basemap->getMap());
}

std::vector<PointCloudPtr> FrameProcessor::processFrame(int frame) {
	std::vector<PointCloudPtr> clouds;
	PointCloudPtr frame_cloud = trace->loadFrame(frame);
	PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f ground_truth = trace->poses[frame];
	applyNoise(ground_truth,0.06);
	PointCloudPtr clipped_map=basemap->clipByRadius(ground_truth, 100);
	pcl::transformPointCloud(*frame_cloud, *frame_cloud, ground_truth);

	//pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt = performNDTAlignment(frame_cloud,clipped_map, output_cloud, prevTransformation, ground_truth);
	//auto finalTransformation = ndt.getFinalTransformation();
	//std::cout<<finalTransformation<<std::endl;
	//pcl::transformPointCloud(*frame_cloud, *frame_cloud, finalTransformation);
	PointCloudPtr appearedPoints= subtract(frame_cloud);
	//PointCloudPtr mergedFrame=basemap->integrateClippedMap(clipped_map,frame_cloud,ground_truth);
	//update prevtransformation to the product of the two transformations
	/*prevTransformation = finalTransformation;*/
	clouds.push_back(appearedPoints);
	clouds.push_back(frame_cloud);
	//clouds.push_back(clipped_map);
	return clouds;
}


PointCloudPtr FrameProcessor::subtract(PointCloudPtr frame) {
// Return what the frame sees that map doesnt have
	PointCloudPtr output(new pcl::PointCloud<pcl::PointXYZ>);
	float radius = 0.1;
	for (int i = 0; i < frame->size(); i++) {
		pcl::PointXYZ point = frame->points[i];
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			if (pointNKNSquaredDistance[0] > radius) {
				output->push_back(point);
			}
		}
	}
	return output;
	
}