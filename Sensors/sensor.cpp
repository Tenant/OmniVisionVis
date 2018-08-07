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
