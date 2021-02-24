#ifndef PGR_H
#define PGR_H

#include <Windows.h>
#include <ladybug.h>
#include <ladybugrenderer.h>
#include <ladybuggeom.h>
#include <ladybugstream.h>

#include <thread>
#include <opencv2/core/core.hpp>

#include "../Sensors/sensor.h"
#include "../Velodyne/velodynedatainterface.h"

#include <string>
using std::string;

#define ResizeLadybugRadio 2.5
#define isRectifyNeeded 0

class LadybugData : public SensorData
{
public:
	LadybugData();
	cv::Mat img;
};

class LadybugConfig : public SensorConfig
{
public:
	virtual bool init(const std::string& path);
	std::string pgrFilename;
	std::string calibFilename;
	double tError_alpha;
	double tError_beta;
	int tError_time;
};

class LadybugReader : public SensorReader
{
private:
	LadybugContext context;
	LadybugStreamContext readContext;
	LadybugStreamHeadInfo streamHeaderInfo;
	unsigned int iTextureWidth, iTextureHeight;
	LadybugImage image;
	unsigned int PANORAMIC_IMAGE_COLUMNS;
	unsigned int PANORAMIC_IMAGE_ROWS;
	//GdiplusStartupInput gdiplusStartupInput;
	//ULONG_PTR gdiplusToken;

	LadybugProcessedImage processedImage;
	long prevTime;
	int cur;
	unsigned int imgnums;//总帧数

#if isRectifyNeeded
	unsigned int RECTIFIED_IMAGE_COLUMNS;
	unsigned int RECTIFIED_IMAGE_ROWS;
	LadybugProcessedImage RectifiedImages[LADYBUG_NUM_CAMERAS];
#endif
public:
	LadybugReader();
	~LadybugReader();
	
	virtual bool init(const std::string& configfile);

	int binarySearchbyTime(const long long t);//返回帧数

	virtual bool grabData(const long long t);
	virtual bool grabNextData();
	virtual bool getTime(long long& t);
	const LadybugData& getCurrentData();
	void VehicleP2ImageP(const cv::Point3d & in, cv::Point2i &out);
	//void ImageP2VehicleP(const cv::Point2i & in, cv::Point3d &out, double distance);
private:
	const cv::Rect _oriROI = cv::Rect(0 * ResizeLadybugRadio, (512-64-64) * ResizeLadybugRadio, 2048 * ResizeLadybugRadio, 256 * ResizeLadybugRadio);

	CoordinateTrans trans;
	LadybugData currentData;
	LadybugConfig config;

	void Local3DPto2DP(const cv::Point3d & in, cv::Point2i &out);

	LadybugError operator()(string file_name);		//载入PGR数据，并初始化为第0帧
	LadybugError operator()(int num);	//指定第num帧
	int	getCur();

	LadybugError outputBMP(string file_name, int num);
	unsigned int cols();	//获取图像宽度
	unsigned int rows();	//获取图像高度

	LadybugProcessedImage getProcessedImage();
	LadybugImage getImage();

	unsigned long int getPgrTime();
	long int getulMicroSeconds();
	time_t getulSeconds();
	LadybugTimestamp getulTimestamp();

	int getulNumberofImages();
#if isRectifyNeeded
	unsigned int rectifiedCols();	//获取图像宽度
	unsigned int rectifiedRows();	//获取图像高度
	LadybugProcessedImage getRectifiedImage(int iCamera);
	LadybugError XZY2RC(double dLadybugX, double dLadybugY, double dLadybugZ, unsigned int  uiCamera,
		double *  pdRectifiedRow, double *  pdRectifiedCol);
#endif
};

#endif