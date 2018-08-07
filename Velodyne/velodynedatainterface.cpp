#include "velodynedatainterface.h"
#include <math.h>
#include <string.h>
#include <ctime>
#include <cmath>
#include <algorithm>

VelodyneReader::VelodyneReader()
{   
    offset = 24;
    frameSize = 1264;
    headerSize = 16;
    skip = 181;
    curFrameIndex = 0;
    scale = 0.001;

    startAngle = 0;
    stopAngle = 360;

	for (auto& datap : _oriVeloData)
		datap = NULL;
}
VelodyneReader::~VelodyneReader()
{
    if (velodyne_file.is_open())
        velodyne_file.close();
	for (auto& datap : _oriVeloData)
		if (datap != NULL)
			delete[] datap;
}

bool VelodyneReader::init(const std::string & path)
{
	if (!config.init(path))
		return false;

	if (!loadCalibParams(config.instrinsicParamsFilename, config.extrinsicParamsFilename))
		return false;

	if (!loadPcapData(config.velodyneFilename))
	{
		return false;
	}
	
	if (!trans.LoadCalib(config.extrinsicParamsFilename))
		return false;
	
	offset = 24;
	frameSize = 1264;
	headerSize = 16;
	skip = 181;
	curFrameIndex = 0;
	scale = 0.001;


	startAngle = 0;
	stopAngle = 360;


	intrinsic = this->calib.getIntrinsicParams();
	extrinsic = this->calib.getExtrinsicParams();

	_oriVeloData.resize(skip);
	for (auto& datap : _oriVeloData)
		datap = new char[frameSize - headerSize];

	return true;
}

/*bool VelodyneReader::getData(int frame, VelodyneData & _data)
{
	char *data = new char[frameSize - headerSize];
	_data.point.clear();
	_data.dist.clear();
	_data.intensi.clear();
	_data.timestamps_gps.clear();
	_data.timestamps_cst.clear();

	long long timestamp;

	for (int k = 0; k<skip; k++) {

		if (velodyne_file.eof())
		{
			if (data)
				delete[] data;
			return false;
		}

		int sec, usec, caplen, len, dlen;
		velodyne_file.read((char*)&sec, sizeof(sec));
		velodyne_file.read((char*)&usec, sizeof(usec));
		velodyne_file.read((char*)&caplen, sizeof(caplen));
		velodyne_file.read((char*)&len, sizeof(len));
		if (len != 1248) {
			dlen = len;
			if (dlen>1248)
				dlen = dlen;
			while (dlen>0) {
				velodyne_file.read(data, std::min(dlen, 1248));
				dlen -= std::min(dlen, 1248);
			}
			k--;
			continue;
		}

		//转为CST时间
		struct tm *p_tm;
		time_t timep(sec);
		p_tm = localtime(&timep);
		timestamp = ((p_tm->tm_hour * 60 + p_tm->tm_min) * 60 + p_tm->tm_sec) * 1000 + usec / 1000;
		_data.timestamps_cst.push_back(timestamp);

		//velodyne_file.read(data, sizeof(char) * (frameSize - headerSize));
		//tahe
		velodyne_file.read(data, sizeof(char) * (frameSize - headerSize - 6));
		velodyne_file.read((char*)&timestamp, sizeof(unsigned int));
		char dumy[2];
		velodyne_file.read((char*)dumy, 2);
		_data.timestamps_gps.push_back(timestamp);

		//塔河？
		ColoredPoint3d onepoint;

		for (int i = 0; i < 12; i++) {
			int blockStart = i * 100 + 42;
			int blockId = (unsigned char)data[blockStart] + (unsigned char)data[blockStart + 1] * 0x100;
			int rotationData = (unsigned char)data[blockStart + 2] + (unsigned char)data[blockStart + 3] * 0x100;

			double start = (startAngle) * 100;
			double stop = (stopAngle) * 100;

			double theta = rotationData / 18000.0 * 3.1415926535;
			for (int j = 0; j < 32; j++) {
				int blockOffset = (blockId == 0xEEFF ? 0 : 32);
				int distanceData = (unsigned char)data[blockStart + 4 + j * 3] + (unsigned char)data[blockStart + 5 + j * 3] * 0x100;
				unsigned char intense = (unsigned char)data[blockStart + 6 + j * 3];

				double distance = distanceData * 2. + intrinsic[j + blockOffset].dist_correction;
				distance *= scale;
				if (distanceData == 0 || distance < MIN_VALID_DISTANCE) {
					onepoint.x = onepoint.y = onepoint.z = 0;
					distance = 0;
					_data.point.push_back(onepoint);
					_data.dist.push_back(distance);
					intense = 0;
					_data.intensi.push_back(intense);
					continue;
				}
				else {

					double cosVertAngle = cos(intrinsic[j + blockOffset].vert_correction);
					double sinVertAngle = sin(intrinsic[j + blockOffset].vert_correction);
					double cosRotCorrection = cos(intrinsic[j + blockOffset].rot_correction);
					double sinRotCorrection = sin(intrinsic[j + blockOffset].rot_correction);

					double cosRotAngle = cos(theta) * cosRotCorrection + sin(theta) * sinRotCorrection;
					double sinRotAngle = sin(theta) * cosRotCorrection - cos(theta) * sinRotCorrection;


					double hOffsetCorr = intrinsic[j + blockOffset].horiz_offset_correction * scale;
					double vOffsetCorr = intrinsic[j + blockOffset].vert_offset_correction * scale;

					double xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

					onepoint.x = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle;
					onepoint.y = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle;
					onepoint.z = distance * sinVertAngle + vOffsetCorr * cosVertAngle;

					//rotatePoint3d(onepoint, OuterCalib.rot_mat);
					//shiftPoint3d(onepoint, OuterCalib.shift);

					_data.point.push_back(onepoint);
					_data.dist.push_back(distance);
					_data.intensi.push_back(intense);
				}
			}
		}
	}
	_data.timestamp = _data.timestamps_cst[0];
	_data.frameNum = curFrameIndex;

	curFrameIndex += 1;// skip; //fyk修改于2018-6-20 12:52:26， 因为不知道原始的有什么用

	if (data)
		delete[] data;
	return true;
}*/

bool VelodyneReader::grabNextData()
{
	currentData.timestamps_gps.clear();
	currentData.timestamps_cst.clear();

	currentData.timestamps_gps.reserve(skip * 12 * 32);//我看下面的循环就是这么读的...
	currentData.timestamps_cst.reserve(skip * 12 * 32);//我看下面的循环就是这么读的...

	long long timestamp;

	std::vector<VelodyneIntrinsicParams> intrinsic = this->calib.getIntrinsicParams();
	VelodyneExtrinsicParams extrinsic = this->calib.getExtrinsicParams();

	for (int k = 0; k<skip; k++) {

		if (velodyne_file.eof())
		{
			if (_oriVeloData[k]!=NULL)
			{
				delete[] _oriVeloData[k];
				_oriVeloData[k] = NULL;
			}
			return false;
		}

		int sec, usec, caplen, len, dlen;
		velodyne_file.read((char*)&sec, sizeof(sec));
		velodyne_file.read((char*)&usec, sizeof(usec));
		velodyne_file.read((char*)&caplen, sizeof(caplen));
		velodyne_file.read((char*)&len, sizeof(len));
		if (len != 1248) {
			dlen = len;
			if (dlen>1248)
				dlen = dlen;
			while (dlen>0) {
				velodyne_file.read(_oriVeloData[k], std::min(dlen, 1248));
				dlen -= std::min(dlen, 1248);
			}
			k--;
			continue;
		}

		//转为CST时间
		struct tm *p_tm;
		time_t timep(sec);
		p_tm = localtime(&timep);
		timestamp = ((p_tm->tm_hour * 60 + p_tm->tm_min) * 60 + p_tm->tm_sec) * 1000 + usec / 1000;
		currentData.timestamps_cst.push_back(timestamp);

		//velodyne_file.read(data, sizeof(char) * (frameSize - headerSize));
		//tahe
		velodyne_file.read(_oriVeloData[k], sizeof(char) * (frameSize - headerSize - 6));
		velodyne_file.read((char*)&timestamp, sizeof(unsigned int));
		char dumy[2];
		velodyne_file.read((char*)dumy, 2);
		currentData.timestamps_gps.push_back(timestamp);
	}

	currentData.timestamp = currentData.timestamps_cst[0];
	currentData.frameNum = curFrameIndex;

	curFrameIndex += 1;// skip; //fyk修改于2018-6-20 12:52:26， 因为不知道原始的有什么用

	return true;
}

bool VelodyneReader::getTime(long long & t)
{
	t = currentData.timestamp;
	return true;
}

const VelodyneData & VelodyneReader::getCurrentData()
{
	currentData.point.clear();
	currentData.dist.clear();
	currentData.intensi.clear();

	currentData.point.reserve(skip * 12 * 32);//我看下面的循环就是这么读的...
	currentData.dist.reserve(skip * 12 * 32);//我看下面的循环就是这么读的...
	currentData.intensi.reserve(skip * 12 * 32);//我看下面的循环就是这么读的...
	for (int k = 0; k<skip; k++)
	{
		//塔河？
		ColoredPoint3d onepoint;

		for (int i = 0; i < 12; i++) {
			int blockStart = i * 100 + 42;
			int blockId = (unsigned char)_oriVeloData[k][blockStart] + (unsigned char)_oriVeloData[k][blockStart + 1] * 0x100;
			int rotationData = (unsigned char)_oriVeloData[k][blockStart + 2] + (unsigned char)_oriVeloData[k][blockStart + 3] * 0x100;

			double start = (startAngle) * 100;
			double stop = (stopAngle) * 100;

			double theta = rotationData / 18000.0 * 3.1415926535;
			for (int j = 0; j < 32; j++) {
				int blockOffset = (blockId == 0xEEFF ? 0 : 32);
				int distanceData = (unsigned char)_oriVeloData[k][blockStart + 4 + j * 3] + (unsigned char)_oriVeloData[k][blockStart + 5 + j * 3] * 0x100;
				unsigned char intense = (unsigned char)_oriVeloData[k][blockStart + 6 + j * 3];

				double distance = distanceData * 2. + intrinsic[j + blockOffset].dist_correction;
				distance *= scale;
				/*if (distanceData == 0 || distance < MIN_VALID_DISTANCE) {
					onepoint.x = onepoint.y = onepoint.z = 0;
					distance = 0;
					currentData.point.push_back(onepoint);
					currentData.dist.push_back(distance);
					intense = 0;
					currentData.intensi.push_back(intense);
					continue;
				}
				else*/
				{

					double cosVertAngle = cos(intrinsic[j + blockOffset].vert_correction);
					double sinVertAngle = sin(intrinsic[j + blockOffset].vert_correction);
					double cosRotCorrection = cos(intrinsic[j + blockOffset].rot_correction);
					double sinRotCorrection = sin(intrinsic[j + blockOffset].rot_correction);

					double cosRotAngle = cos(theta) * cosRotCorrection + sin(theta) * sinRotCorrection;
					double sinRotAngle = sin(theta) * cosRotCorrection - cos(theta) * sinRotCorrection;


					double hOffsetCorr = intrinsic[j + blockOffset].horiz_offset_correction * scale;
					double vOffsetCorr = intrinsic[j + blockOffset].vert_offset_correction * scale;

					double xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

					onepoint.x = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle;
					onepoint.y = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle;
					onepoint.z = distance * sinVertAngle + vOffsetCorr * cosVertAngle;

					trans.LocalP2VehicleP(onepoint, onepoint);

					currentData.point.push_back(onepoint);
					currentData.dist.push_back(distance);
					currentData.intensi.push_back(intense);
				}
			}
		}
	}
	return currentData;
}

void VelodyneReader::setRange(double start_angle, double stop_angle)
{
    this->startAngle = start_angle;
    this->stopAngle = stop_angle;
}

void VelodyneReader::getRange(double& start_angle, double& stop_angle)
{
     start_angle = this->startAngle;
     stop_angle = this->stopAngle;
}

bool VelodyneReader::loadCalibParams(std::string inner_calib_filename)
{
    return calib.loadIntrinsicParams(inner_calib_filename);
}

bool VelodyneReader::loadCalibParams(std::string inner_calib_filename, std::string out_calib_filename)
{
    return calib.loadIntrinsicParams(inner_calib_filename) && calib.loadExtrinsicParams(out_calib_filename);
}

bool VelodyneReader::loadPcapData(std::string filename)
{
    if(filename == "")
        return 0;

    velodyne_file.open(filename.c_str(), std::ios_base::in | std::ios_base::binary);
    if (!velodyne_file.is_open())
        return 0;

	velodyne_file.seekg(24);

    return 1;
}

void VelodyneReader::getPtOrder(std::vector<int> &id)
{
    std::vector<VelodyneIntrinsicParams> InnerCalib = this->calib.getIntrinsicParams();
	
	for (int j = 0; j < InnerCalib.size(); j++)
		id.push_back(InnerCalib[j].id);
}

bool VelodyneConfig::init(const std::string & path)
{
	fs.open(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["veloInstrinsicParamsFilename"] >> instrinsicParamsFilename;
	fs["veloExtrinsicParamsFilename"] >> extrinsicParamsFilename;
	fs["velodyneFilename"] >> velodyneFilename;
	fs.release();
	return true;
}
