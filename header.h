#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#define TBB_EXT_STR "_debug.lib"
#else
#define CV_EXT_STR ".lib"
#define TBB_EXT_STR ".lib"
#endif

/*#pragma comment(lib, "opencv_core"	CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_highgui"	CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_imgproc"	CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_imgcodecs"	CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_videoio" CV_VERSION_STR CV_EXT_STR)*/
#pragma comment(lib, "opencv_world"	CV_VERSION_STR CV_EXT_STR)

#define ymlGetVar(fs, xxx) fs[#xxx] >> xxx

const std::string winNameMono = "mono";
const std::string winNameBV = "bv";
const std::string winNamePano = "pano";

const int _velodyneFPS = 10;
const int _panoWidth = 2048;
const int _panoHeight = 1024;
const int _monoWidth = 1024;
const int _monoHeight = 768;
const int _mapSize = 601;
const double _pixelSize = 0.2;
