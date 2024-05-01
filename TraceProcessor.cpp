#include "TraceProcessor.h"

TraceProcessor::TraceProcessor(Basemap* basemap, Trace* trace){
	this->frameProcessor = new FrameProcessor(basemap, trace);
}

void TraceProcessor::start() {
	int num_frames = frameProcessor->trace->num_frames;
	Timer timer;
	for (int i = 0; i < num_frames; i++) {
		
		timer.start("Frame");
		std::vector<PointCloudPtr> processedFrame = frameProcessor->processFrame(i);
		timer.stop("Frame");
		for (int j = 0; j < processedFrame.size(); j++) {
			frameProcessor->trace->saveFrame(processedFrame[j],i+1,std::to_string(j));
		}
	}
	timer.print();
	frameProcessor->timer.print();
	std::vector<ClusterTrace> clusterTraces = frameProcessor->appearedPointsProcessor->getClusterTraces();
	for (int i = 0; i < clusterTraces.size(); i++) {
		frameProcessor->trace->saveFrame(clusterTraces[i].getMergedCloud(), i, "cluster");
	}
	frameProcessor->basemap->saveMap("merged-6-vehicles-40cmloc.pcd");
}

