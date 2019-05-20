#include "LadybugMask.h"

void LadybugMaskData::clearAll()
{
	uid_local.clear();
	clsNames.clear();
	scores.clear();
	bboxes.clear();
}

bool LadybugMaskConfig::init(const std::string & path)
{
	fs.open(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["MaskRootPath"] >> rootPath;
	fs["MaskTimeFilename"] >> timeFilename;
	fs["MaskMaskPrefix"] >> maskPrefix;
	fs["MaskInfoPrefix"] >> infoPrefix;
	fs["MaskMaskPostfix"] >> maskPostfix;
	fs["MaskInfoPostfix"] >> infoPostfix;
	fs.release();
	return true;
}

bool LadybugMaskReader::init(const std::string & configfile)
{
	if (!config.init(configfile))
		return false;

	curIndex = 0;
	timestamps.clear();
	std::ifstream timestampfile;
	timestampfile.open(config.rootPath + config.timeFilename);

	long time;
	if (!timestampfile.is_open())
	{
		return false;
	}
	while (!timestampfile.eof())
	{
		timestampfile >> time;
		timestamps.push_back(time);
	}
	timestampfile.close();

	return true;
}

bool LadybugMaskReader::grabData(const long long t)
{
	long timeWindow = 1000;
	if (t < timestamps[0] - timeWindow || t > timestamps[timestamps.size() - 1] + timeWindow)
		return false;
	if (!findImageByTime(t, curTime, curIndex))//确定一下超过index会return false
	{
		return false;
	}

	readSingleMaskData();

	return true;
}

bool LadybugMaskReader::grabNextData()
{
	curIndex++;
	if (curIndex >= timestamps.size())
		return false;
	curTime = timestamps[curIndex];

	readSingleMaskData();

	return true;
}

bool LadybugMaskReader::readSingleMaskData()
{
	std::cout << config.rootPath + config.maskPrefix + std::to_string(curIndex) + config.maskPostfix << std::endl;
	currentData.clearAll();
	currentData.timestamp = curTime;
	currentData.mask = cv::imread(config.rootPath + config.maskPrefix + std::to_string(curIndex) + config.maskPostfix);

	std::ifstream inFile;
	inFile.open(config.rootPath + config.infoPrefix + std::to_string(curIndex) + config.infoPostfix);

	long time;
	if (!inFile.is_open())
	{
		return false;
	}
	char comma;
	std::string clsName;
	double score;
	cv::Rect bbox;
	int curUID = 0;
	
	while (!inFile.eof())
	{
		inFile >> clsName >> score >> comma
			>> bbox.x >> comma >> bbox.y >> comma >> bbox.width >> comma >> bbox.height >> comma >> curUID;
		clsName.pop_back();//comma
		bbox.width -= bbox.x;
		bbox.height -= bbox.y;
		currentData.uid_local.push_back(curUID);
		currentData.clsNames.push_back(clsName);
		currentData.scores.push_back(score);
		currentData.bboxes.push_back(bbox);
	}
	inFile.close();
	return true;
}

bool LadybugMaskReader::getTime(long long & t)
{
	t = timestamps[curIndex];
	return true;
}

const LadybugMaskData& LadybugMaskReader::getCurrentData()
{
	return currentData;
}

bool LadybugMaskReader::findImageByTime(long t, long & nearest_time, int & nearest_index)//这个二分查找有点小问题，left和right最后的大小都不一定
{
	int size = timestamps.size();
	if (size < 2)
		return 0;
	int left = 0, right = size - 1;
	int mid;

	while (left < right)
	{
		if (left == right - 1)
			break;

		mid = (left + right) / 2;
		if (timestamps[mid] > t) {
			right = mid;
		}
		else if (timestamps[mid] < t)
		{
			left = mid;
		}
		else //if (timestamps[mid] == t)
		{
			left = right = mid;
			break;
		}
	}
	if (right <= 0 || left >= timestamps.size())
		return 0;

	if (abs(timestamps[left] - t) > abs(t - timestamps[right]))
		nearest_index = right;
	else
		nearest_index = left;

	if (abs(timestamps[nearest_index] - t) <= 100) {
		nearest_time = timestamps[nearest_index];
		return 1;
	}
	else {
		std::cout << "abs(timestamps[nearst_index] - t)<= 100\n";
		nearest_time = timestamps[nearest_index];
		return 1;
		//return 0;
	}
}

