#include "imageShifter.h"


void imageShifter::setShift(int s, const Mat& i)
{
	shift = s;
	image.create(i.size(), i.type());
	for (int c = 0; c < i.cols; c++)
	{
		for (int r = 0; r < i.rows; r++)
		{
			image.at<Vec3b>(r, (c - s + i.cols) % i.cols) = i.at<Vec3b>(r, c);
		}
	}
}

int imageShifter::getShift()
{
	return shift;
}