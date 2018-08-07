#include "getVeloPosFromImage.h"

double pt3dDistance(cv::Point3d p)
{
	return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

bool getRoughPosPano(cv::Rect rect, const VelodyneData& veloData, LadybugReader& ladybug, cv::Point3d& center)
{
	std::vector<cv::Point3d> ptList;
	for (auto vp : veloData.point)
	{
		cv::Point2i lp;
		ladybug.VehicleP2ImageP(vp, lp);
		if (isInImage(lp, rect))
		{
			ptList.push_back(vp);
		}
	}
	if (ptList.empty())
	{
		return false;
	}
	sort(ptList.begin(), ptList.end(), [](cv::Point3d a, cv::Point3d b) -> bool { return pt3dDistance(a) < pt3dDistance(b); });
	cv::Point3d pos = ptList[ptList.size() / 3];
	for (int i = ptList.size() / 3; i < ptList.size() * 2 / 3; i++)
	{
		pos.x += ptList[i].x;
		pos.y += ptList[i].y;
		pos.z += ptList[i].z;
		//dist += sqrt(ptList[i].x*ptList[i].x + ptList[i].y*ptList[i].y + ptList[i].z*ptList[i].z);
	}
	pos.x /= ptList.size() / 3;
	pos.y /= ptList.size() / 3;
	pos.z /= ptList.size() / 3;
	//dist /= ptList.size() / 3;

	center = pos;
	return true;
}

bool getAccuratePosPano(cv::Rect rect, const VelodyneData & veloData, LadybugReader & ladybug, accurateBBox & bbox, double percentage)
{
	if (percentage > 1 || percentage < 0)
		return false;

	std::vector<cv::Point3d> ptList;
	for (auto vp : veloData.point)
	{
		if (vp.z < minValidVeloHeight || vp.z > maxValidVeloHeight)
			continue;
		cv::Point2i lp;
		ladybug.VehicleP2ImageP(vp, lp);
		if (isInImage(lp, rect))
		{
			ptList.push_back(vp);
		}
	}
	if (ptList.empty())
	{
		return false;
	}

	//sort(ptList.begin(), ptList.end(), [](cv::Point3d a, cv::Point3d b) -> bool { return pt3dDistance(a) < pt3dDistance(b); });
	sort(ptList.begin(), ptList.end(), [](cv::Point3d a, cv::Point3d b) -> bool { return a.z < b.z; });
	cv::Point3d pos(0, 0, 0);
	int startIndex = ptList.size() * (0.5 - percentage / 2.0);
	int endIndex = ptList.size() * (0.5 + percentage / 2.0);
	if (endIndex == startIndex)//假如点很少的时候会发生...
		endIndex = startIndex + 1;
	bbox.leftMost = ptList[startIndex];//就算只有一个点，逻辑也对
	bbox.rightMost = ptList[startIndex];
	bbox.topMost = ptList[startIndex];
	bbox.bottomMost = ptList[startIndex];
	bbox.nearMost = ptList[startIndex];
	bbox.farMost = ptList[startIndex];

	for (int i = startIndex; i < endIndex; i++)
	{
		pos.x += ptList[i].x;
		pos.y += ptList[i].y;
		pos.z += ptList[i].z;

		if (ptList[i].x < bbox.leftMost.x)
			bbox.leftMost = ptList[i];
		if (ptList[i].x > bbox.rightMost.x)
			bbox.rightMost = ptList[i];
		if (ptList[i].z < bbox.bottomMost.z)
			bbox.bottomMost = ptList[i];
		if (ptList[i].z > bbox.topMost.z)
			bbox.topMost = ptList[i];
		double dist = pt3dDistance(ptList[i]);
		if (dist < pt3dDistance(bbox.nearMost))
			bbox.nearMost = ptList[i];
		if (dist > pt3dDistance(bbox.farMost))
			bbox.farMost = ptList[i];
	}
	pos.x /= (endIndex - startIndex);
	pos.y /= (endIndex - startIndex);
	pos.z /= (endIndex - startIndex);

	bbox.centerMean = pos;

	return true;
}
