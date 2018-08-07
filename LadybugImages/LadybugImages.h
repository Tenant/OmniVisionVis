#pragma once
#include "../header.h"
#include <string>
#include <vector>
#include <fstream>

class LadybugImages
{
public:
	bool init(std::string timefn, std::string imgfn);
	bool getImageByTime(long t, cv::OutputArray img);
private:
	std::vector<long> timestamps;
	std::string imageFolder;
	bool findImageByTime(long timestamp, long& nearest_time);
};