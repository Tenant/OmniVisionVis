#include <iostream>
#include <string>
#include <vector>
#include "OmniVision.h"

using namespace std;
using namespace cv;

void configWindows(int status) {
	int  cx = GetSystemMetrics(SM_CXFULLSCREEN) - 10;
	int  cy = GetSystemMetrics(SM_CYFULLSCREEN) - 10;
	int pano_width_ = cx - 10;
	int pano_height_ = int(500 * pano_width_ / _panoWidth);
	int mono_height_ = cy - pano_height_ - 10;
	int mono_width_ = int(_monoWidth * mono_height_ / _monoHeight);
	int velo_height_ = mono_height_;
	int velo_width_ = mono_height_;

	switch (status) {
	case 0:
		cv::namedWindow(winNameMono, CV_WINDOW_NORMAL);
		cv::namedWindow(winNameBV, CV_WINDOW_NORMAL);
		cv::namedWindow(winNamePano, CV_WINDOW_NORMAL);
		cv::resizeWindow(winNameMono, cv::Size(mono_width_, mono_height_));
		cv::resizeWindow(winNameBV, cv::Size(velo_width_, velo_height_));
		cv::resizeWindow(winNamePano, cv::Size(pano_width_, pano_height_));
		cv::moveWindow(winNameMono, pano_width_ - velo_width_ - mono_width_ - 10, 0);
		cv::moveWindow(winNameBV, pano_width_ - velo_width_, 0);
		cv::moveWindow(winNamePano, 0, mono_height_ + 10);
		break;
	case 1:
	case 2:
	case 3:
		ShowWindow(GetParent((HWND)cvGetWindowHandle(winNamePano.c_str())), 0);
	}
}

int main()
{
	OmniVision omni;
	if (!omni.init())
		return 0;

	configWindows(omni._refineStep);

	while (omni.getData())
	{
		//omni.testImageP2VehicleP();
		//omni.showLMS();
		omni.showVelo();
		omni.showSavedLabel();

		if (omni._refineStep == 0)
			omni.label();
		else if (omni._refineStep == 1)
			omni.refineMonoLabel();
		else if (omni._refineStep == 2)
			omni.addMissingGlobalBVLabel();
		else if (omni._refineStep == 3)
			omni.refineGlobalBVLabel();

		omni.keyborad();
	}
	omni.release();
	return 0;
}

