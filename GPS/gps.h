#ifndef GPS_H
#define GPS_H

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

//#include "define.h"
#include <string>
#include <fstream>
#include "../header.h"//CV_PI
#include "../Sensors/sensor.h"
#include "../Transform_fang/transform_fang.h"
#include "../Transform_fang/GPS/coordinateconvertion.h"

enum GPS_METHOD { GPS_NEAREST, GPS_INTERPOLATED };

class GPSData : public SensorData
{
public:
	long unixTime;
	double roll;
	double pitch;
	double yaw;
	double x;
	double y;
	double z;
	int svNum[2];
};

class GPSConfig : public SensorConfig
{
public:
	virtual bool init(const std::string& configfile);
	std::string gpsfile;
	std::string imufile;
	std::string calibfile;

	GPS_METHOD method;
};

class GPSReader : public SensorReader
{
public:
	virtual bool init(const std::string& configfile);
	virtual bool grabData(const long long t);
	const GPSData& getCurrentData();
	const GPSData& getPrevData();
	void VehicleP2GlobalP(cv::Point3d& in, cv::Point2d & out);
	void GlobalP2VehicleP(cv::Point2d & in, cv::Point3d& out);
private:
	GPSData getNearestPosition(const long long time);
	GPSData getInterpolatedPosition(const long long time);
public:
	double maxX, minX, maxY, minY;
	CoordinateTrans trans;
private:
	bool isReferencePointSet;
	CoordinateConvertion geographicCoordTrans;
	GPSData currentData;
	GPSData prevData;
	GPSConfig config;
	std::vector<GPSData> position;
};

#endif