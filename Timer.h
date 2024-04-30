#pragma once
#include<chrono>
#include<map>
#include<string>
#include<iostream>
#include<vector>

using namespace std::chrono;
class Timer {
	private:
		std::map<std::string, std::vector<time_point<high_resolution_clock>>> startTimes;
		std::map<std::string, std::vector<time_point<high_resolution_clock>>> endTimes;
	public:
		void start(std::string name);
		void stop(std::string name);
		void print();
};