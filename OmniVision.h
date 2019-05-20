#pragma once
#include "Velodyne/interface_zhao/velo_dsvl.h"
#include "Velodyne/velodynedatainterface.h"
#include "GPS/gps.h"
#include "LadybugImages/LadybugImages.h"
#include "LadybugPGR/LoadLadyBugPgr.h"
#include "LadybugVideo/LadybugVideo.h"
#include "LadybugMask/LadybugMask.h"
#include "Flea2/flea2reader.h"
#include "lms/urg.h"
#include "misc.h"
#include "header.h"
#include "Sensors/sensor.h"
#include "getVeloPosFromImage.h"
#include "TrackedObj/trackedObj.h"

#define isCalib 0
#define isLabel 0

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

	bool keyborad();

private:
	long long _startTime;
	std::vector<SensorReader*> _priSensors;//主传感器，保证不会漏掉任何一帧
	std::vector<SensorReader*> _subSensors;//副传感器

	LadybugReader _ladybug;
	//VelodyneReader _velodyne;
	VeloDSVLReader _velodyne;
	LadybugMaskReader _mask;

	LadybugData _panoData;
	//VelodyneData _veloData;
	VeloDSVLData _veloData;

	LadybugMaskData _maskData;

	cv::VideoWriter _VW_pano;
	cv::VideoWriter _VW_mono;
	cv::VideoWriter _VW_bv;
	cv::VideoWriter _VW_mask;

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