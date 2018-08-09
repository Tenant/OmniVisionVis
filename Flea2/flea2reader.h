#pragma once

#include "../header.h"
#include "../Sensors/sensor.h"
#include "../Transform_fang/transform_fang.h"
#include "../Transform_fang/Flea2/CAL_MAIN.H"
#include <fstream>
#include <algorithm>

class Flea2Data : public SensorData
{
public:
	cv::Mat img;
	std::string filename;
};

class Flea2Config : public SensorConfig
{
public:
	virtual bool init(const std::string& configfile);
	std::string path;
	std::string extrinsicParamsFilename;

	std::string timeFilename;
	std::string imglistFilename;
};

class Flea2Reader : public SensorReader
{
public:
	virtual bool init(const std::string& path);
	virtual bool grabData(const long long t);
	virtual bool grabNextData();
	virtual bool getTime(long long& t);
	const Flea2Data& getCurrentData();

	void VehicleP2ImageP(const cv::Point3d & in, cv::Point2i& out);
	void ImageP2VehicleP(const cv::Point2i & in, cv::Point3d& out);
private:
	int _curIndex;
	CoordinateTrans trans;
	Flea2Data currentData;
	Flea2Config config;
	std::vector<long long> timestamps;
	std::vector<std::string> imgFilenames;

	bool findDataByTime(long t, int & nearest_index, long tolerate_time = -1);//假如有tolerate_time其实直接用stl的binary_search/lower_bound/就好
	void generateTimeFromFilename();
};

std::vector<std::string> splitWithStl(const std::string &str, const std::string &pattern);
