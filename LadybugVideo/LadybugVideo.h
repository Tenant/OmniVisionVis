#pragma once

#include "../header.h"
#include <fstream>
//#include "../LadybugReader/LadybugReader.h"

class LadybugVideo
{
public:
	LadybugVideo();
	~LadybugVideo();
	bool init(std::string t, std::string v);
	bool getImageByTime(long t, cv::OutputArray img);
private:
	int curPos;
	int maxPos;
	cv::VideoCapture vc;
	std::vector<long> timestamps;
	bool findImageByTime(long timestamp, long& nearest_time);
};