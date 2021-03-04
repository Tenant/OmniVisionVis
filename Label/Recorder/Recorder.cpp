#include "Recorder.h"
#include<windows.h>

bool OneGroundTruth::decode(const string& inString)
{
	std::stringstream convertor(inString);
	char comma;

	convertor >> gtID >> comma >> visibleStartTime >> comma >> visibleEndTime >> comma >> sensorType >> comma >> objClass
		>> comma >> comma >> comma
		>> objCorners[0].x >> comma >> objCorners[0].y >> comma
		>> objCorners[1].x >> comma >> objCorners[1].y >> comma
		>> objCorners[2].x >> comma >> objCorners[2].y >> comma
		>> objCorners[3].x >> comma >> objCorners[3].y >> comma >> comma
		>> comma >> comma >> comma >> objPos.x >> comma >> objPos.y >> comma >> comma >> comma
		>> uid >> comma >> visibleMinStartTime >> comma >> visibleMaxEndTime;

	//39, 38462158, 38462158, C, V, "[546, 431, 608, 431, 608, 370, 546, 370]", "[39.7874625872, 116.0002067943]", 9, -3689348814741910324, -3689348814741910324


	if (convertor.fail() == true)
	{
		return false;
	}
	return true;
}

string OneGroundTruth::encode()
{
	std::stringstream convertor;
	convertor.str("");
	convertor << gtID << ", " << visibleStartTime << ", " << visibleEndTime << ", " << sensorType << ", " << objClass
		<< ", \"["
		<< objCorners[0].x << ", " << objCorners[0].y << ", "
		<< objCorners[1].x << ", " << objCorners[1].y << ", "
		<< objCorners[2].x << ", " << objCorners[2].y << ", "
		<< objCorners[3].x << ", " << objCorners[3].y << "]\""
		<< std::setiosflags(std::ios::fixed) << std::setprecision(10) << ", \"[" << objPos.x << ", " << objPos.y << "]\","//经纬度精度至少小数点后10位
		<< uid << ", " << visibleMinStartTime << ", " << visibleMaxEndTime;
	return convertor.str();
}

void OneGroundTruth::draw(cv::Mat & canvas, cv::Scalar color)
{
	cv::line(canvas, objCorners[0], objCorners[1], color);
	cv::line(canvas, objCorners[1], objCorners[2], color);
	cv::line(canvas, objCorners[2], objCorners[3], color);
	cv::line(canvas, objCorners[3], objCorners[0], color);
}

void OneGroundTruth::draw_bv(cv::Mat & canvas, cv::Scalar color)
{
	cv::Point2i dir(0, 0);
	cv::Point2i center(0, 0);
	cv::Point2i bvp[4];
	for (int i = 0; i < 4; i++)
	{
		bvp[i].x = objCorners[i].x / _pixelSize + _mapSize / 2;
		bvp[i].y = _mapSize / 2 - objCorners[i].y / _pixelSize;
		center += bvp[i];
	}
	center /= 4;
	dir = (bvp[2] + bvp[1]) - center;
	
	cv::line(canvas, bvp[2], dir, CV_RGB(255, 0, 0));
	cv::line(canvas, bvp[1], dir, CV_RGB(255, 0, 0));

	cv::line(canvas, bvp[0], bvp[1], CV_RGB(255, 0, 0));
	cv::line(canvas, bvp[1], bvp[2], CV_RGB(255, 0, 0));
	cv::line(canvas, bvp[2], bvp[3], CV_RGB(255, 0, 0));
	cv::line(canvas, bvp[3], bvp[0], CV_RGB(255, 0, 0));
}

double OneGroundTruth::generateYawFromCorners()
{
	double dx, dy;
	//c1是x-y+(左上方)，c2是x+y+(右上方)
	dx = objCorners[2].x - objCorners[1].x;
	dy = objCorners[2].y - objCorners[1].y;
	return atan2(dy, dx);// [-pi,+pi]
}

bool Recorder::load(const string& filename)
{
	gts.clear();

	//要不就全部读完，最后全部一次写，这样代码写起来最方便
	//同时读写的话需要维护文件的指针，这个也不是二进制很麻烦，以后万一想加一个字段就更麻烦
	std::ifstream inputFile;
	inputFile.open(filename);
	if (!inputFile)//假如文件不存在
	{
		std::ofstream createFileTmp(filename);
		if (!createFileTmp)
		{
			return false;
		}
		inputFile.open(filename);
	}
	if (!inputFile.is_open())
	{
		return false;
	}
	OneGroundTruth ogttmp;
	string stmp;
	int max_uid = 0;
	while (!inputFile.eof())
	{
		getline(inputFile, stmp);
		if (ogttmp.decode(stmp))
		{
			if (ogttmp.uid >= max_uid)
				max_uid = ogttmp.uid + 1;
			gts.push_back(ogttmp);//写成这样是为了防止存储文件里面不完整/无序/重复
		}
	}
	inputFile.close();

	saved_max_uid = max_uid;

	return true;
}

void Recorder::update(OneGroundTruth onegt)
{
	gts.push_back(onegt);
}

bool Recorder::erase(char sensorType, int uid, long long t)
{
	for (auto gt = gts.begin(); gt != gts.end();)
	{
		if ((gt->sensorType == sensorType) &&
			(gt->uid == uid) &&
			(gt->visibleStartTime <= t && t <= gt->visibleEndTime)
			)
		{
			gt = gts.erase(gt);
			return true;
		}
		else
		{
			++gt;
		}
	}
	return false;
}

bool Recorder::getSavedGT(char sensorType, long long t, std::vector<OneGroundTruth>& gt)
{
	gt.clear();
	for (auto tmp : gts)
	{
		//假如这里不先排好序就必须全部遍历一遍，应该先按照起始时间排序，后面save的时候重新按照gtID排序
		if (tmp.sensorType != sensorType)
			continue;
		if (isSensorTypeMatched(sensorType, 'C'))
		{
			if (tmp.visibleStartTime == t && t == tmp.visibleEndTime)
			{
				gt.push_back(tmp);
			}
		}
		else if (isSensorTypeMatched(sensorType, 'L'))
		{
			if (tmp.visibleStartTime <= t && t < tmp.visibleEndTime)
			{
				gt.push_back(tmp);
			}
		}
	}
	return !gt.empty();
}

void Recorder::setOneSavedGT(OneGroundTruth onegt, int index)
{
	gts[index] = onegt;
}

OneGroundTruth Recorder::getOneSavedGT(int index)
{
	return gts[index];
}

bool Recorder::getOneSavedGTIndex(char sensorType, int uid, long long t, int& index)//每一帧有多个gt...
{
	int _size = gts.size();
	for (int i = 0; i < _size; i++)
	{
		if (!isSensorTypeMatched(gts[i].sensorType, sensorType))
			continue;
		if (gts[i].uid!= uid)
			continue;
		if (gts[i].visibleStartTime == t)//假的激光起始时间太害人...
		//if (gts[i].visibleStartTime <= t && t < gts[i].visibleEndTime)
		{
			index = i;
			return true;
		}
	}
	return false;
}

void Recorder::generateGTDuration(int curMaxUID)//生成起始时间终止时间,
{
	for (int loop_uid = 0; loop_uid <= curMaxUID; loop_uid++)
	{
		long long starttime = 10e10;//a meaningless large number
		long long endtime = 0;
		std::vector<int> indexOfCurUID;
		indexOfCurUID.clear();
		for (int loop_index = 0; loop_index < gts.size(); loop_index++)
		{
			if (gts[loop_index].uid != loop_uid)
				continue;
			indexOfCurUID.push_back(loop_index);
			if (gts[loop_index].visibleStartTime < starttime)
				starttime = gts[loop_index].visibleStartTime;
			if (gts[loop_index].visibleEndTime > endtime)
				endtime = gts[loop_index].visibleEndTime;
		}
		for (auto loop_index : indexOfCurUID)
		{
			gts[loop_index].visibleMinStartTime = starttime;
			gts[loop_index].visibleMaxEndTime = endtime;
		}
	}
}


bool Recorder::save(const string& filename)
{
	int curMaxUID = 0;
	for (auto gt : gts)
	{
		if (gt.uid > curMaxUID)
			curMaxUID = gt.uid;
	}

	std::ofstream outputFile;
	outputFile.open(filename);
	if (!outputFile.is_open())
	{
		return false;
	}

	generateGTDuration(curMaxUID);
	for (int i = 1; i <= gts.size(); i++)
	{
		gts[i - 1].gtID = i;
		//临时措施
		if (isSensorTypeMatched(gts[i - 1], 'L'))
		{
			gts[i - 1].visibleEndTime = gts[i - 1].visibleStartTime + oneVeloFrameVisibleTimeDuration;
		}
		outputFile << gts[i-1].encode() << std::endl;
	}
	outputFile.close();

	std::cout << "save successfully" << std::endl;
	FILE *fp = fopen("debug.log", "w");
	fprintf(fp, "上次保存时时间戳位：%d\n", _currentTime);
	fclose(fp);
	MessageBox(NULL, "保存成功，本次保存的时间戳已记录到dubug.log文件", "Save Successfully", MB_OK);
	
	return true;
}

bool isSensorTypeMatched(const char sensorType0, const char sensorType1)
{
	return (toupper(sensorType0) == toupper(sensorType1) || tolower(sensorType0) == tolower(sensorType1));
}

bool isSensorTypeMatched(OneGroundTruth& gt0, const char sensorType1)
{
	return isSensorTypeMatched(gt0.sensorType, sensorType1);
}

bool isSensorTypeMatched(OneGroundTruth& gt0, OneGroundTruth& gt1)
{
	return isSensorTypeMatched(gt0.sensorType, gt1.sensorType);
}
