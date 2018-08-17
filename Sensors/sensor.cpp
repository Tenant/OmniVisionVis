#include "sensor.h"

SensorConfig::~SensorConfig()
{
	if(fs.isOpened())
		fs.release();
}

bool SensorConfig::init(const std::string & path)
{
	fs.open(path, cv::FileStorage::READ);
	return fs.isOpened();
}

/*template<typename T>
inline bool SensorConfig::read(std::string name, T & t)
{
	cv::FileNode tmp = fs[name];
	bool isFound = !tmp.isNone();
	if (isFound)
	{
		tmp >> t;
	}
	//else似乎不能同时读写
	//{
	//	fs.write(name, t);
	//}
	return isFound;
}
*/