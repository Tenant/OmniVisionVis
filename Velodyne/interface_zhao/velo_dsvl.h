#pragma once

#include "../../Sensors/sensor.h"
#include "../../header.h"
#include "../../Transform_fang/transform_fang.h"
#include "../veloxmlcalibinterface.h"
#include <unordered_map>

#define	PNTS_PER_LINE		32
#define	LINES_PER_BLK		12
#define	PTNUM_PER_BLK		PNTS_PER_LINE*LINES_PER_BLK
#define	BKNUM_PER_FRM		181
//180的话有一个缝隙
//#define	SCANDATASIZE		(180*12)

typedef struct {
	float			x, y, z;
	uchar			i;
} point3fi;

typedef struct {
	int			millisec;
	point3fi		points[PTNUM_PER_BLK];
} ONEVDNDATA;

typedef struct {
	point3d			ang;
	point3d			shv;
	int			millisec;
	point3fi		points[PTNUM_PER_BLK];
	int				lab[PTNUM_PER_BLK];
} ONEDSVDATA;

typedef struct {
	ONEDSVDATA		dsv[BKNUM_PER_FRM];
	point3d			ang;
	point3d			shv;
	MATRIX			rot;
} ONEDSVFRAME;

class lSeg
{
public:
	lSeg();
	lSeg(int blk_id_, int pt_id_);//添加第一个元素
	int totalPtsNum;
	std::vector<int> blk_id;
	std::vector<int> pt_id;
	int associatedImgSegIndex;
};

class VeloDSVLData : public SensorData
{
public:
	int frameNum;
	ONEDSVFRAME* onefrm;
	std::unordered_map<int, lSeg>	seg;//直接根据label索引所有的点
};

class VeloDSVLConfig : public SensorConfig
{
public:
	virtual bool init(const std::string& path);
	std::string instrinsicParamsFilename;
	std::string extrinsicParamsFilename;
	std::string velodyneFilename;

	double minValidVeloHeight;
	double maxValidVeloHeight;
};

class VeloDSVLReader : public SensorReader
{
public:
	VeloDSVLReader();
	~VeloDSVLReader();
	virtual bool init(const std::string& path);
	virtual bool grabNextData();
	virtual bool getTime(long long& t);
	const VeloDSVLData& getCurrentData();

private:
	bool loadCalibParams(std::string inner_calib_filename, std::string out_calib_filename);
	//bool loadPcapData(std::string filename);

	void jumpTo(int fno);

private:
	CoordinateTrans trans;
	VeloDSVLConfig config;
	VeloDSVLData currentData;

	VeloXMLCalibInterface calib;

	std::ifstream dfp;

	int dFrmNo;

	int dsbytesiz;

	uint64 numOfBytes;// Total Dsv Frame Num
};