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
	unsigned int imgnums;//��֡��

#if isRectifyNeeded
	unsigned int RECTIFIED_IMAGE_COLUMNS;
	unsigned int RECTIFIED_IMAGE_ROWS;
	LadybugProcessedImage RectifiedImages[LADYBUG_NUM_CAMERAS];
#endif
public:
	LadybugReader();
	~LadybugReader();
	
	virtual bool init(const std::string& configfile);

	int binarySearchbyTime(const long long t);//����֡��

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

	LadybugError operator()(string file_name);		//����PGR���ݣ�����ʼ��Ϊ��0֡
	LadybugError operator()(int num);	//ָ����num֡
	int	getCur();

	LadybugError outputBMP(string file_name, int num);
	unsigned int cols();	//��ȡͼ����
	unsigned int rows();	//��ȡͼ��߶�

	LadybugProcessedImage getProcessedImage();
	LadybugImage getImage();

	unsigned long int getPgrTime();
	long int getulMicroSeconds();
	time_t getulSeconds();
	LadybugTimestamp getulTimestamp();

	int getulNumberofImages();
#if isRectifyNeeded
	unsigned int rectifiedCols();	//��ȡͼ����
	unsigned int rectifiedRows();	//��ȡͼ��߶�
	LadybugProcessedImage getRectifiedImage(int iCamera);
	LadybugError XZY2RC(double dLadybugX, double dLadybugY, double dLadybugZ, unsigned int  uiCamera,
		double *  pdRectifiedRow, double *  pdRectifiedCol);
#endif
};

#endif