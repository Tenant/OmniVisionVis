#include <iostream>
#include <string>
#include <vector>
#include "Velodyne/velodynedatainterface.h"
#include "GPS/gpsinterface.h"
#include "LadybugImages/LadybugImages.h"
#include "Ladybug3DPoint/Ladybug3DPoint.h"
#include "LadybugPGR/LoadLadyBugPgr.h"
#include "LadybugVideo/LadybugVideo.h"
#include "misc.h"
#include "header.h"

using namespace std;
using namespace cv;


VelodyneDataInterface _velodyne;
GPSInterface _gps;
//LadybugImages _ladybug;
LadybugVideo _ladybug;
Ladybug3DPoint _ladybugCalib;
PGR _pgr;

const int imgWidth = 2048;
const int imgHeight = 1024;

cv::VideoWriter _VW;


bool initialization()
{
	string instrinsicCalibFilename;
	string extrinsicCalibFilename;
	string ladybugTimeFilename;
	string velodyneFilename;
	string gpsFilename;
	string vcCalibFilename;
	string ladybugPath;
	string pgrFilename;

	cv::FileStorage fs("config.yml", cv::FileStorage::READ);
	fs["instrinsicCalibFilename"] >> instrinsicCalibFilename;
	fs["extrinsicCalibFilename"] >> extrinsicCalibFilename;
	fs["velodyneFilename"] >> velodyneFilename;
	fs["gpsFilename"] >> gpsFilename;
	fs["ladybugPath"] >> ladybugPath;
	fs["PGRFilename"] >> pgrFilename;
	fs["ladybugTimeFilename"] >> ladybugTimeFilename;
	fs["vcCalibFilename"] >> vcCalibFilename;
	fs.release();

	//加载gps
	//if (!_gps.loadGPSData(gpsFilename))
	//	return false;

	//加载velodyne
	if(!_velodyne.loadData(velodyneFilename))
		return false;
	//velodyne.setRange(135, 315);
	_velodyne.loadCalibParams(instrinsicCalibFilename, extrinsicCalibFilename);

	//加载ladybug
	//_pgr(pgrFilename);
	if(!_ladybug.init(ladybugTimeFilename, ladybugPath))
		return false;

	//加载激光和全景相机标定文件
	if(!_ladybugCalib.LoadCalib(vcCalibFilename, imgWidth, imgHeight))
		return false;

	_VW.open("out.avi", CV_FOURCC('X', 'V', 'I', 'D'), 10, cv::Size(imgWidth, imgHeight));

	return true;
}

int main()
{
	if (!initialization())
		return 0;
		
	long long timestamp, timestamp_utc;

	std::vector<ColoredPoint3d> points;
	std::vector<double> dists;

	OneGPSData onegps;
	cv::Mat image(cv::Size(imgWidth, imgHeight), CV_MAKETYPE(CV_8U, 3));
	cv::Mat canvas;

	bool getFrame(std::vector<long long> &timestamp_gps, std::vector<long long> &timestamp_utc, std::vector<ColoredPoint3d> &points, std::vector<double> &dist, std::vector<unsigned char> &intensi);
	while (_velodyne.getFrame(timestamp, timestamp_utc, points, dists, 0))
	{
		if (!_ladybug.getImageByTime(timestamp_utc, image))
			break;

		image.copyTo(canvas);
		cv::cvtColor(image, canvas, CV_BGR2GRAY);
		cv::cvtColor(canvas, canvas, CV_GRAY2BGR);

		for (auto vp : points)
		{
			cv::Point2i lp;
			_ladybugCalib.LVP2LP(vp, lp);
			canvas.at<cv::Vec3b>(lp) = vp.color;
		}

		cv::imshow("calib", canvas);

		_VW << canvas;

		char keyboard = cv::waitKey(10);
		if (keyboard == 27)
		{
			break;
		}
		else if (keyboard == 20)
		{
			cv::waitKey();
		}
	}
	return 0;
}

