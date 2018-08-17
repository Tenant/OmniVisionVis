#pragma once
#include "Velodyne/velodynedatainterface.h"
#include "GPS/gps.h"
#include "LadybugImages/LadybugImages.h"
#include "LadybugPGR/LoadLadyBugPgr.h"
#include "LadybugVideo/LadybugVideo.h"
#include "Flea2/flea2reader.h"
#include "lms/urg.h"
#include "misc.h"
#include "header.h"
#include "Sensors/sensor.h"
#include "GTLabel.h"
#include "getVeloPosFromImage.h"

#define isCalib 0
#define isLabel 1

//激光可见范围40米内
//#define bvVisibleDistance 40
//#define imageVisibleHeight 25

class OmniVision
{
public:
	//图像可见范围40像素以上
	OmniVision() : _imageVisibleHeight(25) {};

	bool init();
	bool getData();
	void release();

	void showLMS();
	void showVelo();
	void showMap();

	void showSavedLabel();
	void label();

	void refineMonoLabel();
	void refineGlobalBVLabel();
	void addMissingGlobalBVLabel();

	bool generateOneGT_withPanoROI(OneGroundTruth& onegt);
	bool generateOneGT_withMonoROI(OneGroundTruth& onegt);
	bool generateOneGT_withBVPoint(OneGroundTruth& onegt);

	void keyborad();

	void testImageP2VehicleP();
private:
	long long _startTime;
	std::vector<SensorReader*> _priSensors;//主传感器，保证不会漏掉任何一帧
	std::vector<SensorReader*> _subSensors;//副传感器

	LadybugReader _ladybug;
	VelodyneReader _velodyne;
	GPSReader _gps;
	Flea2Reader _flea2;
	URGReader _urg;

	LadybugData _panoData;
	VelodyneData _veloData;
	GPSData _gpsData;
	GPSData _gpsPrevData;
	Flea2Data _monoData;
	URGData _urgData;

	GTLabel _gtLabeler;

	cv::VideoWriter _VW_pano;
	cv::VideoWriter _VW_mono;
	cv::VideoWriter _VW_bv;

	cv::Mat _pano_velo;
	cv::Mat _mono_velo;
	cv::Mat _bv_velo;

public:
	int _refineStep;//0 是粗标定， 1 是细标定图像， 2是补全缺失的激光，3是细标定激光及GPS

private:
	int _imageVisibleHeight;

	int _waitKeyTime;
	long long _currentTime;//针对多个主传感器而设置的
	ColorMap _colormap;

	int _refineUID;//需要refine的UID
	std::string _refineObjClass;//需要refine的物体类型
};