#pragma once

#include <string>
#include <vector>
#include "../Transform_fang/transform_fang.h"
#include "../header.h"
#include "intrinsic/velointrinsic.h"
#include "extrinsic/veloextrinsic.h"

#define MIN_VALID_DISTANCE 0.5

class VeloXMLCalibInterface{
    public:
		VeloXMLCalibInterface(){}
        ~VeloXMLCalibInterface(){}

        bool loadIntrinsicParams(std::string filename);
		bool loadExtrinsicParams(std::string filename);

		VeloXMLIntrinsicParams& getIntrinsicParams();
		VelodyneExtrinsicParams& getExtrinsicParams();
    private:
		VeloXMLIntrinsicParams intrinsicParams;
        VelodyneExtrinsicParams extrinsicParams;
};
