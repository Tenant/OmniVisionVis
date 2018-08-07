#pragma once

#include "header.h"
using cv::Mat;
using cv::Vec3b;

class imageShifter
{
public:
	void setShift(int s, const Mat& i);
	int getShift();
	Mat image;
private:
	int shift;
};