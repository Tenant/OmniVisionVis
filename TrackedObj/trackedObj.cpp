#include "trackedObj.h"
#include <ctime>

void trackedObjs::update(cv::Mat img, LadybugMaskData mask)
{
	cv::RNG rng((unsigned)time(NULL));//以后修改为用RNG.nect

	int totalObjNum = mask.scores.size();
	//...
	std::vector<bool> isNewObject(totalObjNum, true);//暂时不作任何判断，直接全部用新的


	//...
	tracks.clear();
	for (int i = 0; i < totalObjNum; i++)
	{
		trackedObj newTrack;
		newTrack.uid_local = mask.uid_local[i];
		newTrack.clsName = mask.clsNames[i];
		std::cout << newTrack.uid_local << " " << newTrack.clsName << std::endl;
		newTrack.score = mask.scores[i];
		newTrack.bbox_2d = mask.bboxes[i];
		newTrack.pos_3d;//之后更新

		if (isNewObject[i])
		{
			//newTrack.color = assignNewColor((unsigned)rng.next());
			switch (newTrack.clsName[0])
			{
			case 'p':
				newTrack.color = cv::Vec3b(255, 0, 0);
				break;
			case 'r':
				newTrack.color = cv::Vec3b(255, 128, 0);
				break;
			case 'c':
				newTrack.color = cv::Vec3b(0, 0, 255);
				break;
			case 'b':
				newTrack.color = cv::Vec3b(255, 0, 128);
				break;
			case 't':
				newTrack.color = cv::Vec3b(0, 255, 0);
				break;
			default:
				newTrack.color = cv::Vec3b(255, 255, 255);
				break;
			}
			newTrack.uid_global = assignNewUID();
		}

		tracks.push_back(newTrack);
	}
}

trackedObj trackedObjs::getTrack(int queryUID)
{
	for (auto it : tracks)
	{
		if (it.uid_local == queryUID)
			return it;
	}
	return trackedObj();
}

void trackedObjs::modifyTrack(int queryUID, int segid)
{
	for (auto& it : tracks)
	{
		if (it.uid_local == queryUID)
		{
			bool found = false;
			for (int sindex = 0 ; sindex < it.lSegID.size(); sindex++)
			{
				if (it.lSegID[sindex] == segid)
				{
					it.lSegID_ptsNum[sindex]++;
					found = true;
					break;
				}
			}
			if (!found)
			{
				it.lSegID.push_back(segid);
				it.lSegID_ptsNum.push_back(1);
			}
		}
	}
}

cv::Vec3b trackedObjs::assignNewColor(int icolor)
{
	return cv::Vec3b(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);

}

int trackedObjs::assignNewUID()
{
	return ++maxUID;
}
