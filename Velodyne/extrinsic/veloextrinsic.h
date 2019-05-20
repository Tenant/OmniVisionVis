#pragma once

#include "../../Transform_fang/transform_fang.h"

class VelodyneExtrinsicParams {
public:
	void setExtrinsicParams(double _shift_x, double _shift_y, double _shift_z, double _rot_x, double _rot_y, double _rot_z);
	point3d shift, rot;
	MATRIX rot_mat;
};