#include "urg.h"

bool URGConfig::init(const std::string& configfile)
{
	fs.open(configfile, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["lmsFilename"] >> lmsFilename;
	fs["lmsExtrinsicParamsFilename"] >> calibFilename;
	fs["isLMSReverse"] >> isReverse;
	fs.release();

	return true;
}

bool URGReader::init(const std::string & path)
{
	if (!config.init(path))
		return false;
	std::ifstream inFile;
	inFile.open(config.lmsFilename, std::ios::binary);
	if (!inFile.is_open())
		return false;
	inFile.read(reinterpret_cast<char *>(&config.angrng), sizeof(float));
	inFile.read(reinterpret_cast<char *>(&config.angres), sizeof(float));
	inFile.read(reinterpret_cast<char *>(&config.unit), sizeof(float));
	config.datasize = (int)(config.angrng / config.angres + 1);

	const int maxCharNumPreLine = 200;
	short * stmp = new short[config.datasize];
	URGRawData dtmp(config.datasize);
	while (!inFile.eof())
	{
		inFile.read(reinterpret_cast<char *>(&dtmp.timestamp), sizeof(long));
		inFile.read(reinterpret_cast<char *>(stmp), sizeof(short)*config.datasize);
		for (int i = 0; i < config.datasize; i++)
		{
			dtmp.data[i] = stmp[i];
		}
		//dtmp.timestamp -= 3600 * 1000; calib的数据不知道为什么时间戳差一个小时
		allData.push_back(dtmp);
	}
	delete[] stmp;
	inFile.close();

	if (!trans.LoadCalib(config.calibFilename))
		return false;

	return true;
}

bool URGReader::grabData(const long long t)
{
	if (findDataByTime(t, curIndex))
	{
		return true;
	}
	return false;
}

const URGData & URGReader::getCurrentData()
{
	curData.timestamp = allData[curIndex].timestamp;
	int ptsNum = allData[curIndex].data.size();
	curData.pts.resize(ptsNum);
	for (int i = 0 ; i < ptsNum; i ++)
	{
		double dist;//meter
		double angle;//radius
		dist = (double)allData[curIndex].data[i] / (double)config.unit;
		if (config.isReverse)
			angle = (ptsNum - i - 1) * config.angres * CV_PI / 180.0;
		else
			angle = i * config.angres * CV_PI / 180.0;

		cv::Point3d localPt;
		localPt.x = dist * cos(angle);
		localPt.y = dist * sin(angle);
		localPt.z = 0;
		
		trans.LocalP2VehicleP(localPt, curData.pts[i]);
	}
	return curData;
}

bool URGReader::findDataByTime(long t, int & nearest_index)
{
	if (allData.empty())
		return false;
	int left = 0, right = allData.size() - 1;
	int mid;

	while (left < right)
	{
		mid = (left + right) / 2;
		if (allData[mid].timestamp > t) {
			right = mid;
		}
		else if (allData[mid].timestamp < t)
		{
			left = mid;
		}
		else //if (timestamps[mid] == t)
		{
			nearest_index = mid;
			return true;
		}
		if (left + 1 == right)
			break;
	}

	if (abs(allData[left].timestamp - t) > abs(t - allData[right].timestamp))
		nearest_index = right;
	else
		nearest_index = left;

	return true;
}