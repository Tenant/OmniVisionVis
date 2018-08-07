#include <fstream>
#include <iostream>
#include <string.h>
#include "velodynecalibinterface.h"

bool VelodyneCalibInterface::loadIntrinsicParams(std::string filename)
{
    if(filename == "")
        return false;

    std::ifstream calibfile;
    calibfile.open(filename.c_str());
    if (!calibfile.is_open())
        return false;

    int id;
    int lineNum;
    calibfile >> lineNum;
    double rot_correction, vert_correction, dist_correction, vert_offset_correction, horiz_offset_correction;
    intrinsicParams.clear();
    for (int i = lineNum - 1; i >= 0; i--) {
        calibfile >> id >> rot_correction >> vert_correction >> dist_correction >> vert_offset_correction >> horiz_offset_correction;
        //std::cout << id << rot_correction << vert_correction << dist_correction << vert_offset_correction <<  horiz_offset_correction;
        intrinsicParams.push_back(VelodyneIntrinsicParams(lineNum, id, rot_correction, vert_correction, dist_correction, vert_offset_correction, horiz_offset_correction));
    }

    calibfile.close();

	return true;
}

bool VelodyneCalibInterface::loadExtrinsicParams(std::string filename)
{
    if(filename == "")
        return false;

    std::ifstream calibfile;
    calibfile.open(filename.c_str());
    if (!calibfile.is_open())
        return false;

    std::string header;
    double rot_x, rot_y, rot_z, shv_x, shv_y, shv_z;
    calibfile>>header>>rot_x>>rot_y>>rot_z;
    calibfile>>header>>shv_x>>shv_y>>shv_z;

    extrinsicParams.setExtrinsicParams(shv_x, shv_y, shv_z, rot_x, rot_y, rot_z);
    calibfile.close();

	return true;
}

VelodyneExtrinsicParams& VelodyneCalibInterface::getExtrinsicParams()
{
    return this->extrinsicParams;
}

std::vector<VelodyneIntrinsicParams>& VelodyneCalibInterface::getIntrinsicParams()
{
    return this->intrinsicParams;
}


