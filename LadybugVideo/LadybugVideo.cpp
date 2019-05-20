#include "LadybugVideo.h"

LadybugVideo::LadybugVideo()
{
	curPos = 0;
}

LadybugVideo::~LadybugVideo()
{
	if (vc.isOpened())
		vc.release();
}

bool LadybugVideo::init(std::string t, std::string v)
{
	vc.open(v);
	if (!vc.isOpened())
		return false;
	std::ifstream timestampfile;
	timestampfile.open(t);
	maxPos = vc.get(cv::CAP_PROP_FRAME_COUNT);

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
	return true;
}

bool LadybugVideo::getImageByTime(long t, cv::OutputArray img)
{
	long index;
	long timeWindow = 1000;
	if (t < timestamps[0] - timeWindow || t > timestamps[timestamps.size() - 1] + timeWindow)
		return false;
	if (!findImageByTime(t, index))//确定一下超过index会return false
	{
		return false;
	}
	if (index > curPos)
	{
		do
		{
			if (vc.grab())
			{
				curPos++;
			}
			else
			{
				return false;
			}
		} while (curPos != index);
	}
	if (!vc.retrieve(img))
		return false;
	return true;
}

bool LadybugVideo::findImageByTime(long t, long & nearest_time)//这个二分查找有点小问题，left和right最后的大小都不一定
{
	int size = timestamps.size();
	if (size < 2)
		return 0;
	int left = 0, right = size - 1;
	int mid, nearst_index;

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
		nearst_index = right;
	else
		nearst_index = left;

	if (abs(timestamps[nearst_index] - t) <= 100) {
		nearest_time = nearst_index;
		return 1;
	}
	else {
		std::cout << "abs(timestamps[nearst_index] - t)<= 100\n";
		nearest_time = nearst_index;
		return 1;
		//return 0;
	}
}
