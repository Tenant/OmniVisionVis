#pragma once

#include <vector>
#include "../header.h"

class SensorData
{
public:
	SensorData() : timestamp(0) {};
	long long timestamp;//CST, Chinese Standard Time
};

class SensorConfig
{
public:
	virtual ~SensorConfig();
	virtual bool init(const std::string& path);
	cv::FileStorage fs;
};

class SensorReader
{
public:
	virtual bool init(const std::string& path) { return false; };
	virtual bool grabData(const long long t) { return false; };
	virtual bool grabNextData() { return false; };
	virtual bool getTime(long long& t) { t = 0; return false; };
	//virtual SensorData getNextData() { return SensorData(); };
	//virtual SensorData getCurrentData() { return SensorData(); };
private:
};