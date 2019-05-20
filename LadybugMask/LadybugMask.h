#pragma once
#pragma once

#include "../header.h"
#include <fstream>
#include "../Sensors/sensor.h"
//#include "../LadybugPGR/LoadLadyBugPgr.h"

class LadybugMaskData : public SensorData
{
public:
	LadybugMaskData() {};
	void clearAll();
	cv::Mat mask;
	std::vector<int> uid_local;
	std::vector<std::string> clsNames;
	std::vector<double> scores;
	std::vector<cv::Rect> bboxes;
	//3d related...
	//cv::Point3d
};

class LadybugMaskConfig : public SensorConfig
{
public:
	virtual bool init(const std::string& path);
	std::string rootPath;
	std::string timeFilename;
	std::string maskPrefix;
	std::string infoPrefix;
	std::string maskPostfix;
	std::string infoPostfix;
};

class LadybugMaskReader : public SensorReader
{
public:
	virtual bool init(const std::string& configfile);
	virtual bool grabData(const long long t);
	virtual bool grabNextData();
	virtual bool getTime(long long& t);
	const LadybugMaskData& getCurrentData();

private:
	LadybugMaskConfig config;
	int curIndex;
	long curTime;
	LadybugMaskData currentData;
	std::vector<long> timestamps;

	bool readSingleMaskData();
	bool findImageByTime(long t, long & nearest_time, int & nearest_index);
};