#include "LadybugImages.h"

bool LadybugImages::getImageByTime(long t, cv::OutputArray img)
{
	long index;
	if (!findImageByTime(t - 7200, index))// velodyne - camera
	//timestamp_utc -= 2000; //  ÉæË®
	//timestamp_utc = timestamp_utc - 4500 - int(count*0.5); //¹æ»®1
	//timestamp_utc = timestamp_utc - 7500 - int(count*0.2);
	// obstacle -1800
	// searching
	{
		return false;
	}

	std::stringstream ss;
	ss << imageFolder << timestamps[index] << ".jpg";
	std::cout << "V:" << t - 7200 << " ;L" << timestamps[index] << std::endl;
	cv::Mat _img = img.getMat();
	cv::Mat tmp = cv::imread(ss.str());
	tmp.copyTo(_img);
	//_img = cv::imread(ss.str());
	if (_img.data = NULL)
		return false;
	return true;
}

bool LadybugImages::init(std::string timefn, std::string imgfn)
{
	imageFolder = imgfn;

	timestamps.clear();
	std::ifstream timestampfile;
	timestampfile.open(timefn.c_str());
	long time;
	if (timestampfile.is_open())
	{
		while (!timestampfile.eof())
		{
			timestampfile >> time;
			timestamps.push_back(time);
		}
		return true;
	}
	return false;
}

bool LadybugImages::findImageByTime(long timestamp, long & nearest_time)
{
	int size = timestamps.size();
	if (size<2)
		return 0;
	int left = 0, right = size - 1;
	int mid, nearst_index;

	while (left < right)
	{
		mid = (left + right) / 2;
		if (timestamps[mid] > timestamp) {
			right = mid - 1;
		}
		else
			left = mid + 1;
	}
	if (right <= 0 || left >= timestamps.size())
		return 0;

	if (timestamps[left] - timestamp > timestamp - timestamps[right])
		nearst_index = right;
	else
		nearst_index = left;

	if (abs(timestamps[nearst_index] - timestamp)< 100) {
		nearest_time = nearst_index;
		return 1;
	}
	else {
		return 0;
	}
}
