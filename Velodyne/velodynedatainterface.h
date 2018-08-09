#ifndef VELODYNEDATAINTERFACE_H
#define VELODYNEDATAINTERFACE_H
#include <fstream>
#include "../Sensors/sensor.h"
#include "../header.h"
#include "../misc.h"
#include "velodynecalibinterface.h"

static double minValidVeloHeight;
static double maxValidVeloHeight;

class VelodyneData : public SensorData
{
public:
	int frameNum;
	std::vector<long long> timestamps_gps;
	std::vector<long long> timestamps_cst;//Chinese Standard Time 
	std::vector<ColoredPoint3d> point;//已经是车体坐标系了
	std::vector<double> dist;
	std::vector<unsigned char> intensi;
};

class VelodyneConfig : public SensorConfig
{
public:
	virtual bool init(const std::string& path);
	std::string instrinsicParamsFilename;
	std::string extrinsicParamsFilename;
	std::string velodyneFilename;
	
	double minValidVeloHeightTemp;
	double maxValidVeloHeightTemp;
};

class VelodyneReader : public SensorReader
{
public:
    VelodyneReader();
    ~VelodyneReader();
	virtual bool init(const std::string& path);
	//virtual bool getData(int frame, VelodyneData & data);
	//virtual bool getData(const SensorData& dt, VelodyneData & data);
	virtual bool grabNextData();
	virtual bool getTime(long long& t);
	const VelodyneData& getCurrentData();

	void setRange(double start_angle, double stop_angle);
	void getRange(double& start_angle, double& stop_angle);
	void getPtOrder(std::vector<int> &id);

private:
    bool loadCalibParams(std::string inner_calib_filename);
    bool loadCalibParams(std::string inner_calib_filename, std::string out_calib_filename);
    bool loadPcapData(std::string filename);

private:
	CoordinateTrans trans;
	VelodyneData currentData;
	VelodyneConfig config;
    VelodyneCalibInterface calib;
	
	std::vector<VelodyneIntrinsicParams> intrinsic;
	VelodyneExtrinsicParams extrinsic;
	
	std::ifstream velodyne_file;
    long long frameNum;
    int curFrameIndex;
    int skip;
    int offset;
    int headerSize;
    int frameSize;

    double startAngle;
    double stopAngle;
    double scale;

	std::vector<char*> _oriVeloData;//原始的velo数据
};

#endif // VELODYNEDATAINTERFACE_H
