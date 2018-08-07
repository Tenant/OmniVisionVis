#pragma once

#include "Velodyne/velodynedatainterface.h"
#include "LadybugPGR/LoadLadyBugPgr.h"

class accurateBBox
{
public:
	cv::Point3d leftMost;
	cv::Point3d rightMost;
	cv::Point3d bottomMost;
	cv::Point3d topMost;
	cv::Point3d nearMost;
	cv::Point3d farMost;
	cv::Point3d centerMean;//按照距离排序中间三分之一点的均值
};

bool getRoughPosPano(cv::Rect rect, const VelodyneData& veloData, LadybugReader& ladybug, cv::Point3d& center);

bool getAccuratePosPano(cv::Rect rect, const VelodyneData& veloData, LadybugReader& ladybug, accurateBBox& bbox, double percentage = 0.33);
