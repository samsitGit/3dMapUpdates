#include "Timer.h"

void Timer::start(std::string name) {
	startTimes[name].push_back(high_resolution_clock::now());
}

void Timer::stop(std::string name) {
	endTimes[name].push_back(high_resolution_clock::now());
	std::cout << name <<" took " << duration_cast<milliseconds>(endTimes[name].back() - startTimes[name].back()).count() << " ms" << std::endl;
}

void Timer::print() {
	for (auto& pair : startTimes) {
		std::string name = pair.first;
		std::vector<time_point<high_resolution_clock>> startTime = pair.second;
		std::vector<time_point<high_resolution_clock>> endTime = endTimes[name];
		std::cout<< name << " captured over " << startTimes[name].size() << " frames" << std::endl;
		for (int i = 0; i < startTimes[name].size(); i++) {
			auto& start = startTime[i];
			auto& end = endTime[i];
			auto duration = duration_cast<milliseconds>(end - start);
			std::cout <<duration.count()<< std::endl;
		}
	}
}