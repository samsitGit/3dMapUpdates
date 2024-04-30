#include "FrameProcessor.h"




FrameProcessor::FrameProcessor(Basemap* basemap, Trace* trace) : basemap(basemap), trace(trace)
{

	appearedPointsProcessor = new ClusterProcessor(3, 6, 15);
	disappearedPointsProcessor = new ClusterProcessor(3, 3, 8);
	pool = new ctpl::thread_pool(8);
	this->aggregated_frames_count = 0;
	this->aggregated_frames = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

}

std::vector<PointCloudPtr> FrameProcessor::processFrame(int frame) {
	std::vector<PointCloudPtr> clouds;
	PointCloudPtr frame_cloud = trace->loadFrame(frame);
	Eigen::Matrix4f ground_truth = trace->poses[frame];
	applyNoise(ground_truth,0.06);
	//PointCloudPtr clipped_map=basemap->clipByRadius(ground_truth, 100);
	pcl::transformPointCloud(*frame_cloud, *frame_cloud, ground_truth);
	timer.start("appeared cluster extraction");
	PointCloudPtr appearedPoints = subtract(frame_cloud);
	std::vector<PointCloudPtr> clusters = extractClusters(appearedPoints);
	timer.stop("appeared cluster extraction");
	for (const auto& cluster : clusters) {
		appearedPointsProcessor->addCluster(Cluster(cluster, frame));
	}
	timer.start("disappeared cluster extraction");
	PointCloudPtr disappearedPoints = vectorRayTraceDisappearedPoints(frame_cloud, basemap->clipByRadius(ground_truth,20), ground_truth, 0.03);
	clusters = extractClusters(disappearedPoints);
	timer.stop("disappeared cluster extraction");
	for (const auto& cluster : clusters) {
		disappearedPointsProcessor->addCluster(Cluster(cluster, frame));
	}

	//std::cout<< "Number of clusters in processor: " << appearedPointsProcessor->getClusterTraces().size() << std::endl;
	timer.start("appeared cluster updates");
	std::vector<Cluster> clusterUpdates = appearedPointsProcessor->getClusterUpdates(frame);
	timer.stop("appeared cluster updates");
	PointCloudPtr addedPointUpdates(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr disappearedPointUpdates(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto& cluster : clusterUpdates) {
		basemap->addCluster(cluster);
		*addedPointUpdates += *cluster.cloud;
	}
	timer.start("disappeared cluster updates");
	clusterUpdates = disappearedPointsProcessor->getClusterUpdates(frame);
	timer.stop("disappeared cluster updates");
	std::cout<<"Number of clusters in processor: "<<clusterUpdates.size()<<std::endl;
	for (auto& cluster : clusterUpdates) {
		//basemap->removeCluster(cluster);
		*disappearedPointUpdates += *cluster.cloud;
	}
	clouds.push_back(addedPointUpdates);
	clouds.push_back(disappearedPoints);
	//clouds.push_back(clipped_map);
	return clouds;
}

std::vector<PointCloudPtr> FrameProcessor::extractClusters(PointCloudPtr cloud) {
	std::vector<PointCloudPtr> clusters;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.1);
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	for (const auto& cluster : cluster_indices) {
		PointCloudPtr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		
		for (const auto& idx : cluster.indices) {
			cloud_cluster->push_back(cloud->points[idx]);
		}
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		clusters.push_back(cloud_cluster);

	}
	return clusters;
}


PointCloudPtr FrameProcessor::subtract(PointCloudPtr frame) {
// Return what the frame sees that map doesnt have
	PointCloudPtr output(new pcl::PointCloud<pcl::PointXYZ>);
	float radius = 0.2;
	for (int i = 0; i < frame->size(); i++) {
		pcl::PointXYZ point = frame->points[i];
		std::vector<NeighborPoint> neighbors = basemap->getNeighbors(point,1);
		if (neighbors[0].distance > radius) {
				output->push_back(point);
		}
	}
	return output;
	
}

std::vector<Eigen::Vector3f> FrameProcessor::vectorRayTracePoint(Eigen::Vector3f lead, NearestVector* nearestVector, float radius) {
	std::vector<Eigen::Vector3f> output;
	std::vector<Eigen::Vector3f> similarVectors = nearestVector->getSimilarVectors(lead, 20);
	float eps = 0.06; //Expected maximum localization error in meters
	//Verify if the projection of a similar vector is radius away from the lead
	for (const auto& vector : similarVectors) {
		//If magnitude of vector is higher than lead, ignore
		if (vector.norm() > lead.norm()-eps) {
			continue;
		}
		//Find projection of vector on lead
		Eigen::Vector3f projection = lead.dot(vector) / lead.dot(lead) * lead;
		//if projection is in opposite direction of lead, ignore
		if (lead.dot(projection) < 0) {
			continue;
		}
		if ((projection - vector).norm() < radius) {
			output.push_back(vector);
		}
	}
	return output;
}

PointCloudPtr FrameProcessor::vectorRayTraceDisappearedPoints(PointCloudPtr frame, PointCloudPtr map, Eigen::Matrix4f origin, float radius) {
	PointCloudPtr output(new pcl::PointCloud<pcl::PointXYZ>);
	//use ctpl to parallelize this
	std::vector<std::future<std::vector<Eigen::Vector3f>>> futures;
	NearestVector* nearestVector = new NearestVector();
	pcl::transformPointCloud(*map, *map, origin.inverse()); // Transform map to frame
	timer.start("indexing vectors");
	for (int i = 0; i < map->size(); i++) {
		pcl::PointXYZ point = map->points[i];
		Eigen::Vector3f vector(point.x, point.y, point.z);
		nearestVector->addVector(vector);
	}
	timer.stop("indexing vectors");
	for (int i = 0; i < frame->size(); i++) {
		pcl::PointXYZ point = frame->points[i];
		futures.push_back(pool->push([this, point, nearestVector,radius](int id) {
			return vectorRayTracePoint(Eigen::Vector3f(point.x, point.y, point.z), nearestVector, radius);
		}));
	}
	for (auto& future : futures) {
		std::vector<Eigen::Vector3f> neighbors = future.get();
		for (const auto& neighbor : neighbors) {
			pcl::PointXYZ pcl_point;
			pcl_point.x = neighbor(0);
			pcl_point.y = neighbor(1);
			pcl_point.z = neighbor(2);
			output->push_back(pcl_point);
		}
	}
	pcl::transformPointCloud(*output, *output, origin); // Transform back to map
	return output;
}

PointCloudPtr FrameProcessor::rayTraceDisappearedPoints(PointCloudPtr frame,Eigen::Matrix4f origin,float resolution) {
	PointCloudPtr output(new pcl::PointCloud<pcl::PointXYZ>);
	//use ctpl to parallelize this
	std::vector<std::future<std::vector<NeighborPoint>>> futures;
	for (int i = 0; i < frame->size(); i++) {
		pcl::PointXYZ point = frame->points[i];
		futures.push_back(pool->push([this, point, origin, resolution](int id) {
			return rayTracePoint(id, point, origin, resolution);
		}));
	}
	int c = 0;
	for (auto& future : futures) {
		c++;
		std::vector<NeighborPoint> neighbors = future.get();
		for (const auto& neighbor : neighbors) {
			output->push_back(basemap->getPoint(neighbor.index));
		}
	}
	return output;

}

std::vector<NeighborPoint> FrameProcessor::rayTracePoint(int id,pcl::PointXYZ point, Eigen::Matrix4f origin, float resolution) {
	std::vector<NeighborPoint> output;
	Eigen::Vector3f origin_point(point.x, point.y, point.z);
	Eigen::Vector3f direction = origin_point - Eigen::Vector3f(origin(0, 3), origin(1, 3), origin(2, 3));
	float distance = direction.norm();
	direction.normalize();
	for (float j = 0; j < distance-resolution*2; j += resolution) {
		Eigen::Vector3f ray_point = origin_point + direction * j;
		pcl::PointXYZ ray_pcl_point;
		ray_pcl_point.x = ray_point(0);
		ray_pcl_point.y = ray_point(1);
		ray_pcl_point.z = ray_point(2);
		std::vector<NeighborPoint> neighbors = basemap->getMapOnlyNeighbors(ray_pcl_point, 1);
		for (const auto& neighbor : neighbors) {
			if (neighbor.distance < resolution) {
				output.push_back(neighbor);
			}
		}
	}
	return output;
}

PointCloudPtr FrameProcessor::subtractMap(PointCloudPtr aggregatedFrame, PointCloudPtr map) {
	// Return what the frame doesnt see that the map has
	PointCloudPtr output(new pcl::PointCloud<pcl::PointXYZ>);
	float radius = 0.2;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(aggregatedFrame);
	for (int i = 0; i < map->size(); i++) {
		pcl::PointXYZ point = map->points[i];
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

NearestVector::NearestVector() {
	this->dim = 3;	
	this->M = 4;
	this->ef_construction = 100;
	this->vectors = std::vector<Eigen::Vector3f>();
	hnswlib::InnerProductSpace ip_space(dim);
	this->hnsw = new hnswlib::HierarchicalNSW<float>(&ip_space, 1000000, M, ef_construction);
}

void NearestVector::addVector(Eigen::Vector3f vector) {
	float* data = new float[3];
	data[0] = vector(0);
	data[1] = vector(1);
	data[2] = vector(2);
	hnsw->addPoint(data, vectors.size());
	vectors.push_back(vector);
}

std::vector<Eigen::Vector3f> NearestVector::getSimilarVectors(Eigen::Vector3f vector, int k) {
	std::vector<Eigen::Vector3f> output;
	float* data = new float[3];
	data[0] = vector(0);
	data[1] = vector(1);
	data[2] = vector(2);
	auto result = hnsw->searchKnn(data, k);
	while (!result.empty()) {
		int index = result.top().second;
		output.push_back(vectors[index]);
		result.pop();
	}
	return output;
}