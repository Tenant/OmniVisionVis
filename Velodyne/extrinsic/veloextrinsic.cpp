#include "veloextrinsic.h"

void VelodyneExtrinsicParams::setExtrinsicParams(double _shift_x, double _shift_y, double _shift_z, double _rot_x, double _rot_y, double _rot_z)
{
	shift.x = _shift_x;
	shift.y = _shift_y;
	shift.z = _shift_z;
	rot.x = _rot_x;
	rot.y = _rot_y;
	rot.z = _rot_z;
	createRotMatrix_XYZ(rot_mat, rot.x, rot.y, rot.z);
}
