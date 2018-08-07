#pragma once

#include "../header.h"
#include "../Sensors/sensor.h"
#include "../Transform_fang/transform_fang.h"
#include <fstream>
#include <algorithm>

class URGData : public SensorData
{
public:
	std::vector<cv::Point3d> pts;
};

class URGRawData : public SensorData
{
public:
	URGRawData() {};
	URGRawData(int dsize) { data.resize(dsize); };
	std::vector<short> data;
};

class URGConfig : public SensorConfig
{
public:
	virtual bool init(const std::string& configfile);

	std::string lmsFilename;
	std::string calibFilename;

	bool isReverse;
	
	float angrng;
	float angres;
	float unit;

	int datasize;
};

class URGReader : public SensorReader
{
public:
	virtual bool init(const std::string& path);
	virtual bool grabData(const long long t);
	const URGData& getCurrentData();
private:
	URGData curData;
	URGConfig config;
	int curIndex;
	std::vector<URGRawData> allData;

	CoordinateTrans trans;

	bool findDataByTime(long t, int & nearest_index);
};