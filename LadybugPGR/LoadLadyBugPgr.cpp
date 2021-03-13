#include "LoadLadyBugPgr.h"


#define COLOR_PROCESSING_METHOD		LADYBUG_HQLINEAR
#define OUTPUT_IMAGE_TYPE			LADYBUG_PANORAMIC

#pragma comment( lib, "ladybug.lib" )

LadybugReader::LadybugReader()
{
	context=NULL;
	readContext=NULL;
	iTextureWidth=0;
	iTextureHeight=0;
	cur=-1;
	PANORAMIC_IMAGE_COLUMNS = 2048 * ResizeLadybugRadio;
	PANORAMIC_IMAGE_ROWS = 1024 * ResizeLadybugRadio;
	//GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
#if isRectifyNeeded
	RECTIFIED_IMAGE_COLUMNS = 808 * ResizeLadybugRadio;
	RECTIFIED_IMAGE_ROWS = 616 * ResizeLadybugRadio;
#endif
}

LadybugReader::~LadybugReader()
{
	if(readContext!=NULL)
	{
		ladybugStopStream(readContext);
		ladybugDestroyStreamContext( &readContext);
		readContext=NULL;
	}
	if(context!=NULL)
	{
		ladybugDestroyContext( &context);
		context=NULL;
	}
	cur=-1;
	//GdiplusShutdown(gdiplusToken);
}

bool LadybugReader::init(const std::string & configfile)
{
	if(!config.init(configfile))
		return false;

	if ((*this)(config.pgrFilename) != LADYBUG_OK)
		return false;

	if (!trans.LoadCalib(config.calibFilename))
		return false;

	return true;
}

const LadybugData& LadybugReader::getCurrentData()
{
	return currentData;
}

int LadybugReader::binarySearchbyTime(const long long t)
{
	int start = 0;
	int end = imgnums - 1;
	unsigned long curTime, startTime, endTime;

	LadybugError error;
	error = ladybugGoToImage(readContext, start);
	if (error != LADYBUG_OK) return -1;
	error = ladybugReadImageFromStream(readContext, &image);
	if (error != LADYBUG_OK) return -1;
	startTime = getPgrTime();
	if (t < startTime) return start;

	error = ladybugGoToImage(readContext, end);
	if (error != LADYBUG_OK) return -1;
	error = ladybugReadImageFromStream(readContext, &image);
	if (error != LADYBUG_OK) return -1;

	endTime = getPgrTime();
	if (t > endTime) return end;
	
	int pos = double(t - startTime) / double(endTime - startTime)*double(imgnums);
	do
	{
		error = ladybugGoToImage(readContext, pos);
		if (error != LADYBUG_OK) return -1;
		error = ladybugReadImageFromStream(readContext, &image);
		if (error != LADYBUG_OK) return -1;
		curTime = getPgrTime();
		if (t > curTime)
		{
			prevTime = curTime;
			return pos;
		}
		pos -= 10;//step=10随便写的
	} while (true);

	;
}

bool LadybugReader::grabData(const long long tt)
{
	int time_adjust = config.tError_beta + int((tt - config.tError_time) * config.tError_alpha);
	long long t = tt - time_adjust;
	int num = cur;
	if (num < 0)//||大于最大值
	{
		return false;
	}

	num = binarySearchbyTime(t);
	if (num == -1)
		return false;

	LadybugError error;
	long curTime = prevTime;
	while (t > curTime)
	{
		prevTime = curTime;
		num++;

		error = ladybugGoToImage(readContext, num);
		if (error != LADYBUG_OK) return error;
		error = ladybugReadImageFromStream(readContext, &image);
		if (error != LADYBUG_OK) return error;

		curTime = getPgrTime();
	}
	if (abs(t - prevTime) < abs(curTime - t))
	{
		num--;
		(*this)(num);
	}
	else
	{
		prevTime = curTime;
	}
	long timeWindow = 100;//100ms内才考虑
	if (abs(t - prevTime) > timeWindow)
	{
		//return LADYBUG_NOT_IMPLEMENTED; //以后再说吧，现在好多时候velodyne先采集
	}

	error = ladybugConvertImage(context, &image, NULL);
	if (error != LADYBUG_OK) return error;
	error = ladybugUpdateTextures(context, LADYBUG_NUM_CAMERAS, NULL);
	if (error != LADYBUG_OK) return error;
	//error = ladybugSetOffScreenImageSize( context, LADYBUG_PANORAMIC, PANORAMIC_IMAGE_COLUMNS,PANORAMIC_IMAGE_ROWS); //只需要第一次设置
	//if( error != LADYBUG_OK ) return error;
	error = ladybugRenderOffScreenImage(context, LADYBUG_PANORAMIC, LADYBUG_BGR, &processedImage);
	if (error != LADYBUG_OK) return error;
	cur = num;

	currentData.timestamp = prevTime;
	cv::Mat fullImg(cv::Size(cols(), rows()), CV_MAKETYPE(CV_8U, 3));
	memcpy(fullImg.data, processedImage.pData, sizeof(uchar)*fullImg.channels()*fullImg.rows*fullImg.cols);
	
	fullImg(_oriROI).copyTo(currentData.img);


	return true;
}

bool LadybugReader::grabNextData()
{
	int num = cur + 1;
	if (num < 0)//||大于最大值
	{
		return false;
	}

	LadybugError error;

	error = ladybugGoToImage(readContext, num);
	if (error != LADYBUG_OK) return error;
	error = ladybugReadImageFromStream(readContext, &image);
	if (error != LADYBUG_OK) return error;

	long curTime = getPgrTime();

	error = ladybugConvertImage(context, &image, NULL);
	if (error != LADYBUG_OK) return error;
	error = ladybugUpdateTextures(context, LADYBUG_NUM_CAMERAS, NULL);
	if (error != LADYBUG_OK) return error;
	//error = ladybugSetOffScreenImageSize( context, LADYBUG_PANORAMIC, PANORAMIC_IMAGE_COLUMNS,PANORAMIC_IMAGE_ROWS); //只需要第一次设置
	//if( error != LADYBUG_OK ) return error;
	error = ladybugRenderOffScreenImage(context, LADYBUG_PANORAMIC, LADYBUG_BGR, &processedImage);
	if (error != LADYBUG_OK) return error;
	cur = num;

	currentData.timestamp = curTime;
	cv::Mat fullImg(cv::Size(cols(), rows()), CV_MAKETYPE(CV_8U, 3));
	memcpy(fullImg.data, processedImage.pData, sizeof(uchar)*fullImg.channels()*fullImg.rows*fullImg.cols);

	fullImg(_oriROI).copyTo(currentData.img);


	return true;
}

bool LadybugReader::getTime(long long & t)
{
	t = currentData.timestamp;
	return true;
}

LadybugError LadybugReader::operator()(string file_name)
{	

	LadybugError error;
	char pszTempConfigName[MAX_PATH];

	error = ladybugCreateContext( &context);
	if( error != LADYBUG_OK ) 
		return error;
	error = ladybugCreateStreamContext(&readContext);
	if( error != LADYBUG_OK ) 
		return error;
	error = ladybugInitializeStreamForReading( readContext, file_name.c_str(),true);
	if( error != LADYBUG_OK ) 
		return error;
	strcpy_s( pszTempConfigName, "temp.cal");
	error = ladybugGetStreamConfigFile( readContext , pszTempConfigName );
	if( error != LADYBUG_OK ) return error;
	error = ladybugLoadConfig( context, pszTempConfigName );
	if( error != LADYBUG_OK ) return error;

	error = ladybugGetStreamNumOfImages(readContext,&imgnums);
	if( error != LADYBUG_OK ) 
		return error;

	//error = ladybugSetColorProcessingMethod(context, LADYBUG_NEAREST_NEIGHBOR_FAST);
	error = ladybugSetColorProcessingMethod(context, LADYBUG_DIRECTIONAL_FILTER);
	if( error != LADYBUG_OK ) return error;
	error = ladybugGetStreamHeader( readContext, &streamHeaderInfo );
	if( error != LADYBUG_OK ) return error;
//	error = ladybugSetColorTileFormat( context, streamHeaderInfo.stippledFormat );
//	if( error != LADYBUG_OK ) return error;
	
	error=ladybugGoToImage(readContext,0);
	if( error != LADYBUG_OK ) return error;

	prevTime = getPgrTime();

	error = ladybugReadImageFromStream( readContext, &image);
	if( error != LADYBUG_OK ) return error;

//	if ( COLOR_PROCESSING_METHOD == LADYBUG_DOWNSAMPLE4 || COLOR_PROCESSING_METHOD == LADYBUG_MONO)
//	{
//		iTextureWidth = image.uiCols / 2;
//		iTextureHeight = image.uiRows / 2;
//	}
//	else 
//	{
		iTextureWidth = image.uiCols;
		iTextureHeight = image.uiRows;
//	}
//	for( int i = 0; i < LADYBUG_NUM_CAMERAS; i++) 
//	{
//		arpTextureBuffers[ i ] = new unsigned char[ iTextureWidth * iTextureHeight * 4  ];
//	}

	error = ladybugInitializeAlphaMasks( context, iTextureWidth, iTextureHeight );
	if( error != LADYBUG_OK ) return error;
	error = ladybugSetAlphaMasking( context, true );
	if( error != LADYBUG_OK ) return error;

	//error = ladybugEnableSoftwareRendering( context, true );
	//if( error != LADYBUG_OK ) return error;

	error = ladybugConfigureOutputImages( context, LADYBUG_PANORAMIC );
	if( error != LADYBUG_OK ) return error;
	error = ladybugSetOffScreenImageSize( context, LADYBUG_PANORAMIC, PANORAMIC_IMAGE_COLUMNS, PANORAMIC_IMAGE_ROWS );  
	if( error != LADYBUG_OK ) return error;

	(*this)(0);//给cur赋值
	return LADYBUG_OK;
}

LadybugError LadybugReader::operator()(int num)
{
	if(num<0||num==cur)
	{
		//return LADYBUG_FAILED;
	}

	LadybugError error;

	error=ladybugGoToImage( readContext, num);
	if( error != LADYBUG_OK ) return error;
	error = ladybugReadImageFromStream( readContext, &image);
	if( error != LADYBUG_OK ) return error;
	error = ladybugConvertImage( context, &image, NULL );
	if( error != LADYBUG_OK ) return error;
	error = ladybugUpdateTextures( context, LADYBUG_NUM_CAMERAS, NULL);
	if( error != LADYBUG_OK ) return error;
	//error = ladybugSetOffScreenImageSize( context, LADYBUG_PANORAMIC, PANORAMIC_IMAGE_COLUMNS,PANORAMIC_IMAGE_ROWS); //只需要第一次设置
	//if( error != LADYBUG_OK ) return error;
	error = ladybugRenderOffScreenImage( context,LADYBUG_PANORAMIC, LADYBUG_BGR16,&processedImage);
	if (error != LADYBUG_OK) return error;
#if isRectifyNeeded
	error = ladybugSetOffScreenImageSize(
		context,
		LADYBUG_ALL_RECTIFIED_IMAGES,
		RECTIFIED_IMAGE_COLUMNS,
		RECTIFIED_IMAGE_ROWS);
	for (int iCamera = 0; iCamera < LADYBUG_NUM_CAMERAS; iCamera++)
	{
		switch (iCamera)
		{
		case 0:
			error = ladybugRenderOffScreenImage(context, LADYBUG_RECTIFIED_CAM0, LADYBUG_BGR, &RectifiedImages[iCamera]);
			break;
		case 1:
			error = ladybugRenderOffScreenImage(context, LADYBUG_RECTIFIED_CAM1, LADYBUG_BGR, &RectifiedImages[iCamera]);
			break;
		case 2:
			error = ladybugRenderOffScreenImage(context, LADYBUG_RECTIFIED_CAM2, LADYBUG_BGR, &RectifiedImages[iCamera]);
			break;
		case 3:
			error = ladybugRenderOffScreenImage(context, LADYBUG_RECTIFIED_CAM3, LADYBUG_BGR, &RectifiedImages[iCamera]);
			break;
		case 4:
			error = ladybugRenderOffScreenImage(context, LADYBUG_RECTIFIED_CAM4, LADYBUG_BGR, &RectifiedImages[iCamera]);
			break;
		case 5:
			error = ladybugRenderOffScreenImage(context, LADYBUG_RECTIFIED_CAM5, LADYBUG_BGR, &RectifiedImages[iCamera]);
			break;
		}
	}
#endif
	cur=num;
	return LADYBUG_OK;
}

LadybugError LadybugReader::outputBMP(string file_name, int num)
{
	if(num<0)
	{
		num=cur;
	}
	if(cur==num)
	{
		return ladybugSaveImage( context, &processedImage, file_name.c_str(), LADYBUG_FILEFORMAT_BMP );
	}
	else
	{
		(*this)(num);
		return ladybugSaveImage( context, &processedImage, file_name.c_str(), LADYBUG_FILEFORMAT_BMP );
	}
}

LadybugProcessedImage LadybugReader::getProcessedImage()
{
	return processedImage;
}
#if isRectifyNeeded
LadybugProcessedImage PGR::getRectifiedImage(int iCamera)
{
	return RectifiedImages[iCamera];
}

LadybugError PGR::XZY2RC(double dLadybugX, double dLadybugY, double dLadybugZ, unsigned int  uiCamera,
	double *  pdRectifiedRow, double *  pdRectifiedCol)
{
	double row, col;
	LadybugError rtn = ladybugXYZtoRC(context, dLadybugX, dLadybugY, dLadybugZ, uiCamera, &row, &col, NULL);
	(*pdRectifiedRow) = row;
	(*pdRectifiedCol) = col;
	return rtn;
}
#endif
LadybugImage LadybugReader::getImage()
{
	return image;
}

unsigned long int LadybugReader::getPgrTime()
{
	const double _timeDelay = 0;// -0.5;// -1.05;

	unsigned long int millisecond;
	unsigned long int time;
	time_t t;
	t = getulSeconds();
	millisecond = getulMicroSeconds();

	LadybugTimestamp tt = getulTimestamp();
	//unsigned long int newms = (tt.ulCycleCount + tt.ulCycleOffset/3072)/ 8; // /8000 * 1000(s/ms)变成毫秒单位

	struct tm * ptm = gmtime(&t);
	time = (((ptm->tm_hour + 8) * 60 + ptm->tm_min) * 60 + ptm->tm_sec) * 1000 + millisecond / 1000;
	int whole_day = 24 * 3600 * 1000;
	if (time >= whole_day) {
		time -= whole_day;
	}
	//time = time + long(1.05 * 1000);
	time = time + long(-_timeDelay * 1000);
	return time;
}

long int LadybugReader::getulMicroSeconds()
{
	//return image.timeStamp.ulCycleSeconds;
	return image.timeStamp.ulMicroSeconds;

}

time_t LadybugReader::getulSeconds()
{

	return image.timeStamp.ulSeconds;
}

LadybugTimestamp LadybugReader::getulTimestamp()
{
	return image.timeStamp;
}

int LadybugReader::getulNumberofImages()
{
	return imgnums;
}


unsigned int LadybugReader::cols()
{
	return processedImage.uiCols;
}

unsigned int LadybugReader::rows()
{
	return processedImage.uiRows;
}
#if isRectifyNeeded
unsigned int PGR::rectifiedCols()
{
	return RECTIFIED_IMAGE_COLUMNS;
}

unsigned int PGR::rectifiedRows()
{
	return RECTIFIED_IMAGE_ROWS;
}
#endif
int LadybugReader::getCur()
{
	return cur;
}

void LadybugReader::Local3DPto2DP(const cv::Point3d & in, cv::Point2i &out)
{
	double longitude, latitude;

	longitude = atan2(in.y, in.x);// [-pi,+pi]
	latitude = asin(in.z);// [-pi/2,+pi/2] 

	longitude = (1 - longitude / CV_PI) / 2;
	latitude = 0.5 - latitude / CV_PI;
	out.x = int(longitude*cols());
	out.y = int(latitude*rows());
	if (out.y < 0)
		out.y = 0;
	else if (out.y >= rows())
		out.y = rows() - 1;
	if (out.x < 0)
		out.x += cols();
	else if (out.x >= cols())
		out.x -= cols();

	out.x -= _oriROI.x;//不放大就不用这个
	out.y -= _oriROI.y;
}

void LadybugReader::VehicleP2ImageP(const cv::Point3d & in, cv::Point2i &out)
{
	cv::Point3d tmp;
	trans.VehicleP2LocalP(in, tmp);
	normalPoint3d(tmp);
	Local3DPto2DP(tmp, out);
}

/*
void LadybugReader::ImageP2VehicleP(const cv::Point2i & in, cv::Point3d &out, double distance, double height)
{
	//需要知道一个distance, 因为尺度估计不出来
	double longitude, latitude;
	longitude = in.x / cols();
	latitude = asin()
}*/

LadybugData::LadybugData()
{
}

bool LadybugConfig::init(const std::string & path)
{
	fs.open(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["PGRFilename"] >> pgrFilename;
	fs["terror_linear"] >> tError_alpha;
	fs["terror_const"] >> tError_beta;
	fs["terror_time_ori"] >> tError_time;
	fs["ladybugCalibFilename"] >> calibFilename;
	fs.release();
	return true;
}
