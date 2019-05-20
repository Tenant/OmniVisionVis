#pragma once

#include "../header.h"
#include "../LadybugMask/LadybugMask.h"

class trackedObj
{
public:
	int uid_global;//在整个跟踪过程中的UID
	int uid_local;//在当前检测结果中单帧的UID
	std::string clsName;
	double score;
	cv::Rect bbox_2d;
	cv::Point2d pos_3d;
	cv::Vec3b color;
	//...
	//appearance features
	//position features
	std::vector<int> lSegID;
	std::vector<int> lSegID_ptsNum;
public:
	//...
	//dist function
private:
};

class trackedObjs
{
public:
	trackedObjs() : maxUID(-1) {};
	void update(cv::Mat img, LadybugMaskData mask);
	trackedObj getTrack(int queryUID);
	void modifyTrack(int queryUID, int segid);

	//void associateLiDARSeg();

public:
private:
	cv::Vec3b assignNewColor(int icolor);
	int assignNewUID();
private:
	int maxUID;
public://test
	std::vector<trackedObj> tracks;
};