#include "velo_dsvl.h"

bool VeloDSVLConfig::init(const std::string & path)
{
	fs.open(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["veloInstrinsicParamsFilename"] >> instrinsicParamsFilename;
	fs["veloExtrinsicParamsFilename"] >> extrinsicParamsFilename;
	fs["velodyneFilename"] >> velodyneFilename;

	fs["minValidVeloHeight"] >> minValidVeloHeight;
	fs["maxValidVeloHeight"] >> maxValidVeloHeight;

	fs.release();
	return true;
}

VeloDSVLReader::VeloDSVLReader()
{
	currentData.onefrm = NULL;
	currentData.onefrm = new ONEDSVFRAME();
}

VeloDSVLReader::~VeloDSVLReader()
{
	if (currentData.onefrm != NULL)
		delete currentData.onefrm;
	if (dfp.is_open())
		dfp.close();
}

bool VeloDSVLReader::init(const std::string & path)
{
	if (!config.init(path))
		return false;

	if (!loadCalibParams(config.instrinsicParamsFilename, config.extrinsicParamsFilename))
		return false;

	if (!trans.LoadCalib(config.extrinsicParamsFilename))
		return false;

	dfp.open(config.velodyneFilename.c_str(), std::ios_base::binary);
	if (!dfp.is_open()) {
		printf("File open failure : %s\n", config.velodyneFilename.c_str());
		return false;
	}

	dFrmNo = 0;
	dsbytesiz = sizeof(ONEDSVDATA);

	//move pointer to end
	dfp.seekg(0, std::ios::end);
	numOfBytes = dfp.tellg() / 180 / dsbytesiz;//std::cout << "Total Dsv Frame Num: " << numOfBytes << std::endl;

	//将文件指针移动到文件开头
	dfp.clear();
	dfp.seekg(0, std::ios::beg);

	return true;
}

void VeloDSVLReader::jumpTo(int fno)
{
	uint64 li;
	//jump frame
	li = (dsbytesiz)*BKNUM_PER_FRM*fno;
	dfp.seekg(li, std::ios::beg);
	dFrmNo = fno;
}

bool VeloDSVLReader::grabNextData()
{

	int		i;
	currentData.onefrm->ang.x = currentData.onefrm->ang.y = currentData.onefrm->ang.z = 0;
	currentData.onefrm->shv.x = currentData.onefrm->shv.y = currentData.onefrm->shv.z = 0;

	for (i = 0; i < BKNUM_PER_FRM; i++) {
		dfp.read((char *)&currentData.onefrm->dsv[i], dsbytesiz);

		if (dfp.gcount() != dsbytesiz)
			break;

		for (int j = 0; j < PTNUM_PER_BLK; j++)
		{
			cv::Point3d onepoint;
			onepoint.x = currentData.onefrm->dsv[i].points[j].x;
			onepoint.y = currentData.onefrm->dsv[i].points[j].y;
			onepoint.z = currentData.onefrm->dsv[i].points[j].z;

			trans.LocalP2VehicleP(onepoint, onepoint);

			currentData.onefrm->dsv[i].points[j].x = onepoint.x;
			currentData.onefrm->dsv[i].points[j].y = onepoint.y;
			currentData.onefrm->dsv[i].points[j].z = onepoint.z;
		}

		/*currentData.onefrm->ang.x += currentData.onefrm->dsv[i].ang.x;
		currentData.onefrm->ang.y += currentData.onefrm->dsv[i].ang.y;
		currentData.onefrm->ang.z += currentData.onefrm->dsv[i].ang.z;
		currentData.onefrm->shv.x += currentData.onefrm->dsv[i].shv.x;
		currentData.onefrm->shv.y += currentData.onefrm->dsv[i].shv.y;
		currentData.onefrm->shv.z += currentData.onefrm->dsv[i].shv.z;*/
	}

	//每一帧求一个平均的位姿，用于数据关联
	/*currentData.onefrm->ang.x /= BKNUM_PER_FRM;//角度也能算平均？？
	currentData.onefrm->ang.y /= BKNUM_PER_FRM;
	currentData.onefrm->ang.z /= BKNUM_PER_FRM;
	currentData.onefrm->shv.x /= BKNUM_PER_FRM;
	currentData.onefrm->shv.y /= BKNUM_PER_FRM;
	currentData.onefrm->shv.z /= BKNUM_PER_FRM; */

	//printf("x:%f,y:%f,z:%f\n", currentData.onefrm->shv.x, currentData.onefrm->shv.y, currentData.onefrm->shv.z);

	currentData.timestamp = currentData.onefrm->dsv[0].millisec;

	++dFrmNo;
	currentData.frameNum = dFrmNo;

	if (i < BKNUM_PER_FRM)
		return 0;
	else
		return 1;

	return false;
}

bool VeloDSVLReader::getTime(long long & t)
{
	t = currentData.timestamp;
	return true;
}

const VeloDSVLData & VeloDSVLReader::getCurrentData()
{
	return currentData;
}

bool VeloDSVLReader::loadCalibParams(std::string inner_calib_filename, std::string out_calib_filename)
{
	return calib.loadIntrinsicParams(inner_calib_filename) && calib.loadExtrinsicParams(out_calib_filename);
}

lSeg::lSeg()
{
	totalPtsNum = 0;
	blk_id.clear();
	pt_id.clear();
	associatedImgSegIndex = -1;
}

lSeg::lSeg(int blk_id_, int pt_id_)
{
	totalPtsNum = 1;
	blk_id.push_back(blk_id_);
	pt_id.push_back(pt_id_);
	associatedImgSegIndex = -1;
}
