#pragma once
#include <iomanip>
#include "Label/Labeler/Labeler.h"
#include "Label/Recorder/Recorder.h"
#include "Label/Tracker_KCF/kcftracker.hpp"

#include "LadybugPGR/LoadLadyBugPgr.h"
#include "Flea2/flea2reader.h"
#include "Velodyne/velodynedatainterface.h"
#include "GPS/gps.h"

#include "getVeloPosFromImage.h"
#include "Label/GTClassSizeInfo/GTClassSizeInfo.h"

#include "imageShifter.h"

//����verification��ͼ�����Ŵ�С
#define verificationMinSize 150

#define verificationWinName "verification"

const std::string TrackingStatusNames[4] = { "Off", "Re-init", "Refine", "Exit" };
enum TrackingStatus { TS_OFF, TS_REINIT, TS_TRACKING, TS_EXIT, TS_INIT};

class GTLabel
{
public:
	bool init();

	bool getRefineSavedGT_timeDuration(int uid, long long curtime);
	void refineSavedGT_bv_addArchor(GPSReader& gps, VelodyneData& veloData, int uid);
	void refineSavedGT_bv_addFakeArchor(GPSReader& gps, VelodyneData& veloData, int uid, cv::Mat& canvas_bv, Flea2Reader& flea2);
	void refineSavedGT_bv_startSailing(GPSReader& gps, int uid);
	void addMissingGT_bv(GPSReader& gps, Flea2Reader& flea2, VelodyneData& veloData, int uid, GTClassInfo gtci);
	void refineSavedGT_mono(Flea2Reader& flea2, GPSReader& gps, Flea2Data& monoData, VelodyneData& veloData, cv::Mat& canvas_mono, cv::Mat& canvas_bv, bool& isAddNew, int uid);
	void visualiseLastArchor(cv::Mat& canvas, GPSReader& gps);


	void visualizeSavedGT(Flea2Reader& flea2, GPSReader& gps, Flea2Data& monoData, VelodyneData& veloData, cv::Mat& canvas_mono, cv::Mat& canvas_bv);
	void labelSingleFrame(LadybugReader& ladybug, Flea2Reader& flea2, GPSData& gpsData, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv);
	void addNewGT(OneGroundTruth onegt);



	bool isTrackerInit() { return tracker.isInit; };
	cv::Rect getTRect() { return tracker.roi; };
	GTClassInfo _curInfo;//ͬʱ��ע���������Ҫ�޸�

	double getLocalYaw(const GPSData& gps);
	double setGlobalYaw(double localYaw, const GPSData& gps);

	int getCurUID() { return tracker.uid; };
	//int getNewUID() { return curMaxUID++; };

	cv::Mat _verification;//�ӿ��޸�
private:
	int _responseTime;//waitkey time
	void interaction(accurateBBox abbox, double localYaw, Flea2Reader& flea2, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv, TrackingStatus& lastStatus);

	int curMaxUID;

	imageShifter shiftedImageForTracking;

	void verify(const cv::Mat& m);//ͬʱ��ע���������Ҫ�޸�
	void generateImageForVerification(const cv::Mat& m, cv::Mat& out);
	void printOnMat(cv::Mat& canvas);

	KCFTrackerWithFlag tracker;		//ͬʱ��ע���������Ҫ�޸�
	TrackingStatus tracker_flag;	//ͬʱ��ע���������Ҫ�޸�
	//double localYaw;				//ͬʱ��ע���������Ҫ�޸�
	double globalYaw;				//ͬʱ��ע���������Ҫ�޸�

	Recorder recorder;
	Labeler labeler;

	std::string outputFile;//��ֵ�ļ�
	
	std::ofstream extraOutputFile;//��ladybug����ϵ�°�Χ��Ҳ������

	long long _refineStartTime;//����ʱ����ڲ����е���ֵ��Ҫ���ɳ��� //
	long long _refineEndTime;
	std::map<long long, cv::Point2d> _mapArchorGPSPositions;//���ڲ�ֵ



	void modifyOneSavedGT(OneGroundTruth& gt, cv::Point2d refinedCenter, GPSReader& gps);
};