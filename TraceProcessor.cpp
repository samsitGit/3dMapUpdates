#include "TraceProcessor.h"

TraceProcessor::TraceProcessor(Basemap* basemap, Trace* trace){
	this->frameProcessor = new FrameProcessor(basemap, trace);
}

void TraceProcessor::start() {
	for (int i = 0; i < frameProcessor->trace->num_frames; i++) {
		std::vector<PointCloudPtr> processedFrame = frameProcessor->processFrame(i);
		for (int j = 0; j < processedFrame.size(); j++) {
			frameProcessor->trace->saveFrame(processedFrame[j],i+1,std::to_string(j));
		}
		std::cout << "Processed frame " << i << std::endl;
	}
}

