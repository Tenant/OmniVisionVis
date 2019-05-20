#pragma once

#include <vector>
#include <algorithm>
#include <string>

#include "tinyxml2.h"
using namespace tinyxml2;

#ifndef _M_PI
#define _M_PI 3.141592653589793
#endif

class VeloXMLIntrinsicParams {
public:
	VeloXMLIntrinsicParams();
	bool loadCalib(std::string filename);
public:
	int linenum; /**< �������� */
	double unit; /**< һ��λ��ʾ��ʵ�ʳ��� */
	double position_x; /**< ����x�����ƽ���� */
	double position_y; /**< ����y�����ƽ���� */
	double position_z; /**< ����z�����ƽ���� */
	double orientation_r; /**< ����roll�������ת */
	double orientation_p; /**< ����pitch�������ת */
	double orientation_y; /**< ����yaw�������ת */
	// intrinsic paras
	std::vector<bool> enabled; /**< ����ĳ�����Ƿ���� */
	std::vector<bool> intensity; /**< ����ǿ���Ƿ���Ч */
	std::vector<double> minIntensity; /**< ��С����ǿ�� */
	std::vector<double> maxIntensity; /**< �����ǿ�� */
	std::vector<double> rotCorrection; /**< ��תƫ�������μ�Velodyne�ĵ��������Ҹ� */
	std::vector<double> vertCorrection; /**< ����ƫ�������μ�Velodyne�ĵ��������¸� */
	std::vector<double> distCorrection; /**< ����ƫ�������μ�Velodyne�ĵ� */
	std::vector<double> distCorrectionX; /**< ����x����ƫ�������μ�Velodyne�ĵ� */
	std::vector<double> distCorrectionY; /**< ����y����ƫ�������μ�Velodyne�ĵ� */
	std::vector<double> vertOffsetCorrection; /**< �������ƫ�������μ�Velodyne�ĵ� */
	std::vector<double> horizOffsetCorrection; /**< �������ƫ�������μ�Velodyne�ĵ� */
	std::vector<double> focalDistance; /**< ���࣬�μ�Velodyne�ĵ� */
	std::vector<double> focalSlope; /**< ����б�ʣ��μ�Velodyne�ĵ� */
	double minAngle; /**< ��С�����Ƕ� */
	double maxAngle; /**< ������Ƕ� */
	std::vector<size_t> idx; /**< �������µļ�����������0-63 */
};
