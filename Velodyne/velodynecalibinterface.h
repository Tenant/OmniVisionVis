#ifndef VELODYNECALIBINTERFACE_H
#define VELODYNECALIBINTERFACE_H

#include <string>
#include <vector>
#include "../Transform_fang/transform_fang.h"
#include "../header.h"
#include "extrinsic/veloextrinsic.h"

#define MIN_VALID_DISTANCE 0.5

class VelodyneIntrinsicParams{

    public:
        VelodyneIntrinsicParams(int total, int _id, double _rot_correction, double _vert_correction, double _dist_correction, double _vert_offset_correction, double _horiz_offset_correction)
        {
            id = _id;
            rot_correction = _rot_correction / 180.0 * 3.1415926535;
            vert_correction = _vert_correction / 180.0 * 3.1415926535;
            dist_correction = _dist_correction;
            vert_offset_correction = _vert_offset_correction;
            horiz_offset_correction = _horiz_offset_correction;
        }

        int id;
        double rot_correction;
        double vert_correction;
        double dist_correction;
        double vert_offset_correction;
        double horiz_offset_correction;
        double theta, phi;
};

class VelodyneCalibInterface{
    public:
        VelodyneCalibInterface(){}
        ~VelodyneCalibInterface(){}

        bool loadIntrinsicParams(std::string filename);
		bool loadExtrinsicParams(std::string filename);
		
		
		std::vector<VelodyneIntrinsicParams> & getIntrinsicParams();

        VelodyneExtrinsicParams& getExtrinsicParams();
    private:
        std::vector<VelodyneIntrinsicParams> intrinsicParams;
        VelodyneExtrinsicParams extrinsicParams;
};

#endif // VELODYNECALIBINTERFACE_H
