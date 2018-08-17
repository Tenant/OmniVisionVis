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

//����ɼ���Χ40����
//#define bvVisibleDistance 40
//#define imageVisibleHeight 25

class OmniVision
{
public:
	//ͼ��ɼ���Χ40��������
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
	std::vector<SensorReader*> _priSensors;//������������֤����©���κ�һ֡
	std::vector<SensorReader*> _subSensors;//��������

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
	int _refineStep;//0 �Ǵֱ궨�� 1 ��ϸ�궨ͼ�� 2�ǲ�ȫȱʧ�ļ��⣬3��ϸ�궨���⼰GPS

private:
	int _imageVisibleHeight;

	int _waitKeyTime;
	long long _currentTime;//��Զ���������������õ�
	ColorMap _colormap;

	int _refineUID;//��Ҫrefine��UID
	std::string _refineObjClass;//��Ҫrefine����������
};