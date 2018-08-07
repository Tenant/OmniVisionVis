#pragma once
#include "header.h"

class ColorMap
{
	//http://www.kennethmoreland.com/color-advice/
public:
	cv::Vec3b getColor(double value, double min, double max);
private:
	const int maxCNum = 1024;
	static const cv::Vec3b colorTable[1024];
};

class ColoredPoint3d : public cv::Point3d
{
public:
	cv::Vec3b color;
};

bool isInImage(const cv::Point2i& pt, const cv::Rect& rect);
bool isInImage(const cv::Point2i& pt, const cv::Mat& img);

bool isValidROI(cv::Rect input, cv::Size region);
cv::Rect cutValidROI(cv::Rect input, cv::Size region);