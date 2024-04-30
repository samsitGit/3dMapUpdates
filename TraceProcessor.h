#include "Trace.h"
#include "FrameProcessor.h"
#include "Timer.h"

using namespace std::chrono;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class TraceProcessor {
	private:
		FrameProcessor* frameProcessor;
	public:
		TraceProcessor(Basemap* basemap, Trace* trace);
		void start();
};