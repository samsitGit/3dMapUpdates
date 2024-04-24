#include "Trace.h"
#include "FrameProcessor.h"
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class TraceProcessor {
	private:
		FrameProcessor* frameProcessor;
	public:
		TraceProcessor(Basemap* basemap, Trace* trace);
		void start();
};