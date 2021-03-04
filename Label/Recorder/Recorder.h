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

//��ʱ��ʩ
#define oneVeloFrameVisibleTimeDuration 100

class OneGroundTruth
{
public:
	bool decode(const string& inString);
	string encode();

	int gtID;						//��1��ʼ������ţ�����ΪΨһ��ʶ����û��ʵ�ʺ��塣
	long long visibleStartTime;		//ԭʼ��������֡��ʱ�������Ψһ������ͬ��֡���ͣ�ͼ��򼤹⣩ʱ�����ͬ��ͬһ֡�г��ֶ�������Ŀ��ʱ��ʱ�������ͬ��
	long long visibleEndTime;		//����ͼ�����ͬ����ʱ��������ڼ��������������Чɨ��������ʱ���
	char sensorType;				//����֡���ͣ���Ӧʹ���������͵Ĵ�������⣬��C���������L�������״��ע�������ִ�Сд�������ܳ���c��l����
	char objClass;					//Ŀ�����ͣ���⵽��Ŀ������࣬��H���ˣ���V����ͨ���ߣ���B�����ӣ���O���ϰ����ע�������ִ�Сд��
	cv::Point2d objCorners[4];		//Ϊһ�����η�������ϽǺ����½����ꡣ����ͼ������ϵ������ԭ��Ϊͼ�����Ͻǣ����ڼ�������ϵ������ԭ��Ϊ����ԭ�㡣��velodyne����ϵ��˳ʱ��
	double objYaw;					//����Χ���Ƿ������
	cv::Point2d objPos;				//Ŀ��λ�ã�Ŀ����ȫ������ϵ�µ�׼ȷλ�ã�����ϵ��GPS����ϵ����һ�£�ΪWGS84����ϵ�µľ�γ�ȡ�
	int uid;						//Ŀ����ţ���Ϊ�����Ŀ���Ψһ��ʶ�룬��ͬһĿ����ŵ���ֵ��Ӧ��Ϊͬһ���壨�Ʒ�ʱ���õ���
	long long visibleMinStartTime;	//��ʼʱ�䣺��Ŀ���һ�������ɼ����ɼ�⣩��ʱ��������������跢�ͣ��Ʒ�ʱ���õ���
	long long visibleMaxEndTime;	//����ʱ�䣺��Ŀ�����һ�������ɼ����ɼ�⣩��ʱ��������������跢�ͣ��Ʒ�ʱ���õ���

public:
	//void generateCorners(); Ҫ���ò�ͬ�������ϵת�������������������3d boxȻ�󷵻����м���
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

	vector<OneGroundTruth> gts;//��...���˷ų�����
	long long _currentTime;
private:
	void generateGTDuration(int curMaxUID);//������ʼʱ����ֹʱ��

	int saved_max_uid;//��ʼ����ʱ������ɣ�д���������������Ϊ��һ�δ��������ܶ಻ͬ��UID�ˣ������಻֪��֮ǰ����ЩUID
};
