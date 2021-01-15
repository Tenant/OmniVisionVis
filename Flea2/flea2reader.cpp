#include "flea2reader.h"

bool Flea2Config::init(const std::string& configfile)
{
	std::string calibFilename;

	fs.open(configfile, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["flea2Path"] >> path;
	fs["flea2CalibFilename"] >> calibFilename;
	fs["flea2ExtrinsicParamsFilename"] >> extrinsicParamsFilename;
	fs.release();

	imglistFilename = path + std::string("timestamp.csv");
	//timeFilename = std::string();

	if (!LoadCameraCalib(calibFilename.c_str()))
		return false;

	return true;
}

bool Flea2Reader::init(const std::string& path)
{
	if (!config.init(path))
		return false;
	if (!trans.LoadCalib(config.extrinsicParamsFilename))
		return false;

	imgFilenames.clear();
	std::ifstream inFile;
	inFile.open(config.imglistFilename);
	if (!inFile.is_open())
		return false;
	const int maxCharNumPreLine = 200;
	char * str = new char[maxCharNumPreLine];

	if (std::isalpha(static_cast<unsigned char>(inFile.peek())))//假如第一行是字母说明是徐总写的格式说明
		inFile.getline(str, maxCharNumPreLine);

	const char * split = ", ";//原始的pos文件都是','分隔符，但是我用excel的直接拷贝出去所以是'\t'分隔符

	while (!inFile.eof())
	{
		inFile.getline(str, maxCharNumPreLine);
		if (strlen(str) == 0)//一旦用excel打开就会添加空行
			break;
		//std::vector<std::string> terms = splitWithStl(str, ",");
		//if (terms.size()!=2)//timestamp, filename
		//	return false;
		//timestamps.push_back(std::stoll(terms[0]));
		//imgFilenames.push_back(terms[1]);
		long  long timebuffer;
		char * p = strtok(str, split);
		sscanf(p, "%lld", &timebuffer);
		p = strtok(NULL, split);
		timestamps.push_back(timebuffer);
		imgFilenames.push_back(p);
	}
	//generateTimeFromFilename();
	inFile.close();

	_curIndex = 0;

	return true;
}

bool Flea2Reader::grabData(const long long t)
{
	if (findDataByTime(t, _curIndex))
	{
		currentData.timestamp = timestamps[_curIndex];
		currentData.filename = config.path + imgFilenames[_curIndex];
		return true;
	}
	return false;
}

bool Flea2Reader::grapPreviousData()
{
	--_curIndex;
	currentData.timestamp = timestamps[_curIndex];
	currentData.filename = config.path + imgFilenames[_curIndex];
	return true;
}

bool Flea2Reader::grabNextData()
{
	++_curIndex;
	currentData.timestamp = timestamps[_curIndex];
	currentData.filename = config.path + imgFilenames[_curIndex];
	return true;
}

bool Flea2Reader::getTime(long long & t)
{
	t = currentData.timestamp;
	return true;
}

const Flea2Data & Flea2Reader::getCurrentData()
{
	currentData.img = cv::imread(currentData.filename);
	assert(currentData.img.data != NULL);
	return currentData;	// TODO: 在此处插入 return 语句
}

void Flea2Reader::VehicleP2ImageP(const cv::Point3d & in, cv::Point2i& out)
{
	cv::Point3d localp;
	//trans.VehicleP2LocalP(in, localp);//其实就是换回激光坐标系，这里其实有歧义
	localp = in;//将标定参数写进.camera

	double	xw, yw, zw;
	double	ix, iy;

/*	xw = localp.x * 1000.0;					//convert the unit to millimeter
	yw = localp.y * 1000.0;
	zw = localp.z * 1000.0;
	WC2IC(xw, yw, zw, &ix, &iy);
	out.x = (int)ix;
	out.y = currentData.img.rows - 1 - (int)iy;*/
	WC2IC_fang(localp.x, localp.y, localp.z, &ix, &iy);
	out.x = (int)ix;
	out.y = (int)iy;
}

void Flea2Reader::ImageP2VehicleP(const cv::Point2i & in, cv::Point3d & out)
{
	IC2WC_fang(&out.x, &out.y, out.z, in.x, in.y);
}

bool Flea2Reader::findDataByTime(long t, int & nearest_index, long tolerate_time)
{
	if (timestamps.empty())
		return false;
	int left = 0, right = timestamps.size() - 1;
	int mid;

	while (left < right)
	{
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
			nearest_index = mid;
			return true;
		}
		if (left + 1 == right)
			break;
	}

	if (abs(timestamps[left] - t) > abs(t - timestamps[right]))
		nearest_index = right;
	else
		nearest_index = left;

	if (tolerate_time >= 0 && abs(timestamps[nearest_index] - t) > tolerate_time)
		return false;
	return true;
}

std::vector<std::string> splitWithStl(const std::string &str, const std::string &pattern)
{
	std::vector<std::string> resVec;

	if ("" == str)
	{
		return resVec;
	}
	//方便截取最后一段数据
	std::string strs = str + pattern;

	size_t pos = strs.find(pattern);
	size_t size = strs.size();

	while (pos != std::string::npos)
	{
		std::string x = strs.substr(0, pos);
		resVec.push_back(x);
		strs = strs.substr(pos + 1, size);
		pos = strs.find(pattern);
	}

	return resVec;
}

long long parseTimeFromFilename(const std::string& s)
{
	if (s.size() < 4)//-4: ".jpg"
		return false;
	std::vector<std::string> terms = splitWithStl(s.substr(0, s.size()-4), "_");
	if (terms.empty())
		return false;
	int ssize = terms.size();
	std::string& hour = terms[ssize - 4];
	std::string& min = terms[ssize - 3];
	std::string& sec = terms[ssize - 2];
	std::string& msec = terms[ssize - 1];
	return ((stoi(hour) + 8) * 3600 + (stoi(min)) * 60 + stoi(sec)) * 1000 + stoi(msec);//+8是因为王超采集的是UTC
}

void Flea2Reader::generateTimeFromFilename()
{
	timestamps.reserve(imgFilenames.size());
	auto push = [this](const std::string& s) {this->timestamps.push_back(parseTimeFromFilename(s)); };
	std::for_each(imgFilenames.begin(), imgFilenames.end(), push);
}

