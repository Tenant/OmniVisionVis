//449068600,49225900,-0.011345,-0.019530,-6.137119,25638.027,-1609.702,-76.325,3,3
#include "gps.h"

bool GPSReader::init(const std::string& configfile)
{
	if (!config.init(configfile))
		return false;

	trans.LoadCalib(config.calibfile);

	isReferencePointSet = false;
	position.clear();

	std::string fileType = config.gpsfile.substr(config.gpsfile.size() - 3, 3);
	if (fileType == "pos")
	{
		std::ifstream inFile(config.gpsfile);
		if (!inFile.is_open())
			return false;

		maxX = -10e5;
		minX = +10e5;
		maxY = -10e5;
		minY = +10e5;

		char str[200];
		const char * split = ",\t";//原始的pos文件都是','分隔符，但是我用excel的直接拷贝出去所以是'\t'分隔符
		while (!inFile.eof())
		{
			inFile.getline(str, 200);
			if (strlen(str) < 1)
				break;
			GPSData buffer;
			char * p = strtok(str, split);
			sscanf(p, "%ld", &buffer.unixTime);
			p = strtok(NULL, split);
			long timebuffer;
			sscanf(p, "%ld", &timebuffer);
			buffer.timestamp = timebuffer;
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.roll);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.pitch);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.yaw);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.x);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.y);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.z);
			p = strtok(NULL, split);
			sscanf(p, "%d", &buffer.svNum[0]);
			p = strtok(NULL, split);
			sscanf(p, "%d", &buffer.svNum[1]);
			position.push_back(buffer);
			if (buffer.x > maxX)
				maxX = buffer.x;
			if (buffer.x < minX)
				minX = buffer.x;
			if (buffer.y > maxY)
				maxY = buffer.y;
			if (buffer.y < minY)
				minY = buffer.y;
		}
		inFile.close();
	}
	else if (fileType == "csv")
	{
		std::ifstream gpsFile(config.gpsfile);
		if (!gpsFile.is_open())
			return false;
		std::ifstream imuFile(config.imufile);
		if (!imuFile.is_open())
			return false;

		maxX = -10e5;
		minX = +10e5;
		maxY = -10e5;
		minY = +10e5;

		char str[200];
		gpsFile.getline(str, 200);//第一行说明
		imuFile.getline(str, 200);//第一行说明

		const char * split = ",\t";//原始的pos文件都是','分隔符，但是我用excel的直接拷贝出去所以是'\t'分隔符
		while (!gpsFile.eof() && !imuFile.eof())
		{
			gpsFile.getline(str, 200);
			if (strlen(str) < 1)
				break;
			GPSData buffer;
			char * p;
			p = strtok(str, split);
			sscanf(p, "%lld", &buffer.timestamp);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.x);// lattitude
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.y);// longitude
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.z);
			p = strtok(NULL, split);
			sscanf(p, "%d", &buffer.svNum[0]);
			p = strtok(NULL, split);
			sscanf(p, "%d", &buffer.svNum[1]);

			if (!isReferencePointSet)//把第一帧GPS经纬度设置为参考点
			{
				geographicCoordTrans.SetCoordinateParameters(CoordinateConvertion::_SELFDEF, buffer.x, buffer.y);
				isReferencePointSet = true;
			}

			double xyz[3], lbh[3];
			lbh[0] = buffer.x;
			lbh[1] = buffer.y;
			lbh[2] = buffer.z;
			//geographicCoordTrans.LocalLBH_XYZ(lbh, xyz);
			geographicCoordTrans.GPS84LBH_LocalXYZ(lbh, xyz);
			
			//double checkLBH[3];
			//geographicCoordTrans.LocalXYZ_LBH(xyz, checkLBH);
			buffer.x = xyz[1];//这边是反的...
			buffer.y = xyz[0];
			buffer.z = xyz[2];

			//徐总测试huaru
			//test << std::setprecision(10) << "LBH, " << lbh[0] << ", " << lbh[1] << ", " << lbh[2];
			//test << std::setprecision(10) << ", XYZ, " << buffer.x << ", " << buffer.y << ", " << buffer.z << std::endl;


			imuFile.getline(str, 200);
			if (strlen(str) < 1)
				break;
			long long ttmp;
			p = strtok(str, split);
			sscanf(p, "%lld", &ttmp);
			if (ttmp != buffer.timestamp)
			{
				std::cout << "asynchronous gps imu\n";
				break;
			}
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.yaw);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.pitch);
			p = strtok(NULL, split);
			sscanf(p, "%lf", &buffer.roll);

			buffer.yaw = buffer.yaw * CV_PI / 180.0;
			buffer.pitch = buffer.pitch * CV_PI / 180.0;
			buffer.roll = buffer.roll * CV_PI / 180.0;

			position.push_back(buffer);
			if (buffer.x > maxX)
				maxX = buffer.x;
			if (buffer.x < minX)
				minX = buffer.x;
			if (buffer.y > maxY)
				maxY = buffer.y;
			if (buffer.y < minY)
				minY = buffer.y;
		}

		gpsFile.close();
		imuFile.close();
	}
	return true;
}

bool GPSReader::grabData(const long long t)
{
	prevData = currentData;
	if (config.method == GPS_NEAREST)
	{
		currentData = getNearestPosition(t);
	}
	else
	{
		currentData = getInterpolatedPosition(t);
	}
	return true;
}

const GPSData & GPSReader::getCurrentData()
{
	return currentData;
}

const GPSData & GPSReader::getPrevData()
{
	return prevData;
}

void GPSReader::VehicleP2GlobalP(cv::Point3d & in, cv::Point2d & out)
{
	cv::Point3d localp;
	trans.VehicleP2LocalP(in, localp);

	MATRIX rot;
	createRotMatrix_XYZ(rot, 0, 0, -currentData.yaw);
	rotatePoint3d(localp, rot);

	point3d sh;
	sh.x = currentData.x;
	sh.y = currentData.y;
	sh.z = currentData.z;
	shiftPoint3d(localp, sh);

	double xyz[3], lbh[3];
	// ①XYZ[0] (East、単位：米)
	// ②XYZ[1] (North、単位：米)
	// ③XYZ[2] (標高、単位：米)

	xyz[0] = localp.y;//反的
	xyz[1] = localp.x;
	xyz[2] = localp.z;
	geographicCoordTrans.LocalXYZ_GPS84LBH(xyz, lbh);
	//geographicCoordTrans.LocalXYZ_LBH(xyz, lbh);
	out.x = lbh[0];
	out.y = lbh[1];
	//out.z = lbh[2];
}

void GPSReader::GlobalP2VehicleP(cv::Point2d & in, cv::Point3d & out)
{
	double xyz[3], lbh[3];
	// ①XYZ[0] (East、単位：米)
	// ②XYZ[1] (North、単位：米)
	// ③XYZ[2] (標高、単位：米)

	double height = currentData.z;
	lbh[0] = in.x;
	lbh[1] = in.y;
	lbh[2] = height;
	
	geographicCoordTrans.GPS84LBH_LocalXYZ(lbh, xyz);
	//geographicCoordTrans.LocalLBH_XYZ(lbh, xyz);

	cv::Point3d localp(xyz[1], xyz[0], xyz[2]);

	point3d sh;
	sh.x = -currentData.x;
	sh.y = -currentData.y;
	sh.z = -currentData.z;
	shiftPoint3d(localp, sh);

	MATRIX rot;
	createRotMatrix_XYZ(rot, 0, 0, currentData.yaw);
	rotatePoint3d(localp, rot);

	trans.LocalP2VehicleP(localp, out);
}

GPSData GPSReader::getNearestPosition(const long long time)
{
	if (time < (*position.begin()).timestamp)
		return (*position.begin());
	if (time >(position[position.size()-2]).timestamp)//因为下面要对比it和(it+1)
		return (*position.rbegin()); 
	for (auto it = position.begin(); it != position.end(); it++)
	{
		if ((*it).timestamp <= time && time < (*(it + 1)).timestamp)
		{
			return (*it);
		}
	}
	return (*position.rbegin());
}

GPSData GPSReader::getInterpolatedPosition(const long long time)
{
	if (time < (*position.begin()).timestamp)
		return (*position.begin());
	if (time >(position[position.size() - 2]).timestamp)//因为下面要对比it和(it+1)
		return (*position.rbegin());
	for (auto it = position.begin(); it != position.end(); it++)
	{
		GPSData pos, nextpos;
		pos = (*it);
		nextpos = (*(it + 1));
		if ((*it).timestamp <= time && time < (*(it + 1)).timestamp)
		{
			double factor = double(time - (*it).timestamp) / double((*(it + 1)).timestamp - (*it).timestamp);
			GPSData buffer;
			buffer.timestamp = (1 - factor) * pos.timestamp + factor * nextpos.timestamp;
			buffer.unixTime = (1 - factor) * pos.unixTime + factor * nextpos.unixTime;
			//角度不能这么直接插值，因为2*PI = 0
			if (abs(pos.roll - nextpos.roll) > CV_PI)
			{
				if (pos.roll > nextpos.roll)
					nextpos.roll += 2 * CV_PI;
				else
					pos.roll += 2 * CV_PI;
			}
			buffer.roll = (1- factor) * (*it).roll + factor * (*(it + 1)).roll;
			if (abs(pos.pitch - nextpos.pitch) > CV_PI)
			{
				if (pos.pitch > nextpos.pitch)
					nextpos.pitch += 2 * CV_PI;
				else
					pos.pitch += 2 * CV_PI;
			}
			buffer.pitch = (1 - factor) * (*it).pitch + factor * (*(it + 1)).pitch;
			if (abs(pos.yaw - nextpos.yaw) > CV_PI)
			{
				if (pos.yaw > nextpos.yaw)
					nextpos.yaw += 2 * CV_PI;
				else
					pos.yaw += 2 * CV_PI;
			}
			buffer.yaw = (1 - factor) * (*it).yaw + factor * (*(it + 1)).yaw;
			buffer.x = (1 - factor) * (*it).x + factor * (*(it + 1)).x;
			buffer.y = (1 - factor) * (*it).y + factor * (*(it + 1)).y;
			buffer.z = (1 - factor) * (*it).z + factor * (*(it + 1)).z;
			buffer.svNum[0] = buffer.svNum[1] = -1;
			return (*it);
		}
	}
	return (*position.rbegin());
}

bool GPSConfig::init(const std::string & configfile)
{
	std::string s_method;
	method = GPS_NEAREST;
	fs.open(configfile, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["gpsFilename"] >> gpsfile;
	fs["imuFilename"] >> imufile;
	fs["gpsMethod"] >> s_method;
	fs["gpsExtrinsicParamsFilename"] >> calibfile;
	if (s_method == "GPS_NEAREST")
		method = GPS_NEAREST;
	else if (s_method == "GPS_NEAREST")
		method = GPS_INTERPOLATED;
	fs.release();
	return true;
}

double shortAngleDist(double a0, double a1) {
	double max = CV_2PI;
	double da = (a1 - a0);
	while(da > max)//da %= max
		da -= max;
	double doubleda = 2 * da;//2 * da 
	while (doubleda > max)//2 * da %= max
		doubleda -= max;
	return doubleda - da;
}

GPSData getInterpolatedGPS_YawXY(long long time, GPSData prev, GPSData next)
{
	//cv::Point2d getInterpolatedPosition(long long time, long long st, long long et, cv::Point2d sp, cv::Point2d ep)
		if (time < prev.timestamp)
			return prev;
		if (time > next.timestamp)
			return next;
		double factor = double(time - prev.timestamp) / double(next.timestamp - prev.timestamp);

		GPSData ip;
		ip.timestamp = time;

		cv::Point2d buffer;
		ip.x = (1 - factor) * prev.x + factor * next.x;
		ip.y = (1 - factor) * prev.y + factor * next.y;

		//角度插值参考：https://gist.github.com/shaunlebron/8832585
		double prevYaw = prev.yaw;
		double nextYaw = next.yaw;
		ip.yaw = prevYaw + shortAngleDist(prevYaw, nextYaw)*factor;

		std::cout << factor << " " << ip.yaw << std::endl;

		return ip;
}
