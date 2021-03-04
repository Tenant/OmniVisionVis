#pragma once
#include "../../header.h"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>

using std::string;
using std::vector;
using cv::Rect;
using cv::Mat;
using cv::Point2i;
using cv::Size;

//临时措施
#define oneVeloFrameVisibleTimeDuration 100

class OneGroundTruth
{
public:
	bool decode(const string& inString);
	string encode();

	int gtID;						//从1开始递增编号，仅作为唯一标识符，没有实际含义。
	long long visibleStartTime;		//原始关联数据帧的时间戳。不唯一，当不同的帧类型（图像或激光）时间戳相同或同一帧中出现多个待检测目标时，时间戳会相同。
	long long visibleEndTime;		//对于图像就是同样的时间戳，对于激光就是真正的有效扫描线最后的时间戳
	char sensorType;				//关联帧类型：对应使用哪种类型的传感器检测，用C（相机）或L（激光雷达）标注，不区分大小写（即可能出现c或l）。
	char objClass;					//目标类型：检测到的目标的种类，用H（人）、V（交通工具）、B（箱子）、O（障碍物）标注，不区分大小写。
	cv::Point2d objCorners[4];		//为一个矩形方框的左上角和右下角坐标。对于图像坐标系，坐标原点为图像左上角；对于激光坐标系，坐标原点为激光原点。（velodyne坐标系）顺时针
	double objYaw;					//【包围框是否带方向】
	cv::Point2d objPos;				//目标位置：目标在全球坐标系下的准确位置，坐标系与GPS坐标系保持一致，为WGS84坐标系下的经纬度。
	int uid;						//目标序号：作为被检测目标的唯一标识码，即同一目标序号的真值对应的为同一物体（计分时会用到）
	long long visibleMinStartTime;	//开始时间：该目标第一次完整可见（可检测）的时间戳（参赛方不需发送，计分时会用到）
	long long visibleMaxEndTime;	//结束时间：该目标最后一次完整可见（可检测）的时间戳（参赛方不需发送，计分时会用到）

public:
	//void generateCorners(); 要调用不同类的坐标系转换？还是这里仅仅生成3d box然后返回自行计算
	void draw(cv::Mat& canvas, cv::Scalar color);
	void draw_bv(cv::Mat& canvas, cv::Scalar color);
	double generateYawFromCorners();

};

bool isSensorTypeMatched(const char sensorType0, const char sensorType1);
bool isSensorTypeMatched(OneGroundTruth& gt0, const char sensorType1);
bool isSensorTypeMatched(OneGroundTruth& gt0, OneGroundTruth& gt1);

class Recorder
{
public:
	bool load(const string& filename);

	void update(OneGroundTruth onegt);
	bool erase(char sensorType, int uid, long long t);

	bool save(const string& filename);

	int getSavedMaxUID() { return saved_max_uid; };

	bool getSavedGT(char sensorType, long long t, std::vector<OneGroundTruth>& gt);

	bool getOneSavedGTIndex(char sensorType, int uid, long long t, int& index);
	OneGroundTruth getOneSavedGT(int index);
	void setOneSavedGT(OneGroundTruth onegt, int index);

	vector<OneGroundTruth> gts;//唉...算了放出来吧
	long long _currentTime;
private:
	void generateGTDuration(int curMaxUID);//生成起始时间终止时间

	int saved_max_uid;//初始化的时候得生成，写在这个类里面是因为上一次储存下来很多不同的UID了，其他类不知道之前有哪些UID
};
