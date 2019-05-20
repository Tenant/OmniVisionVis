#include <fstream>
#include <iostream>
#include <string.h>
#include "VeloXMLCalibInterface.h"

bool VeloXMLCalibInterface::loadIntrinsicParams(std::string filename)
{
    if(filename == "")
        return false;

	return intrinsicParams.loadCalib(filename);
}

bool VeloXMLCalibInterface::loadExtrinsicParams(std::string filename)
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

VelodyneExtrinsicParams& VeloXMLCalibInterface::getExtrinsicParams()
{
    return this->extrinsicParams;
}

VeloXMLIntrinsicParams& VeloXMLCalibInterface::getIntrinsicParams()
{
    return this->intrinsicParams;
}


