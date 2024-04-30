#include "TraceProcessor.h"

TraceProcessor::TraceProcessor(Basemap* basemap, Trace* trace){
	this->frameProcessor = new FrameProcessor(basemap, trace);
}

void TraceProcessor::start() {
	PointCloudPtr allmerged = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	int num_frames = frameProcessor->trace->num_frames;
	//int num_frames = 100;
	Timer timer;
	for (int i = 0; i < num_frames; i++) {
		
		timer.start("Frame");
		std::vector<PointCloudPtr> processedFrame = frameProcessor->processFrame(i);
		timer.stop("Frame");
		for (int j = 0; j < processedFrame.size(); j++) {
			*allmerged += *processedFrame[j];
			frameProcessor->trace->saveFrame(processedFrame[j],i+1,std::to_string(j));
		}
	}
	timer.print();
	frameProcessor->timer.print();
	std::vector<ClusterTrace> clusterTraces = frameProcessor->appearedPointsProcessor->getClusterTraces();
	for (int i = 0; i < clusterTraces.size(); i++) {
		frameProcessor->trace->saveFrame(clusterTraces[i].getMergedCloud(), i, "cluster");
	}
	frameProcessor->trace->saveFrame(allmerged, frameProcessor->trace->num_frames, "merged");
}

