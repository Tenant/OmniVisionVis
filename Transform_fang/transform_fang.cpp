#include "transform_fang.h"
#include <cmath>

void rMatrixInit(MATRIX &rt)
{
	for (int i = 0; i<3; i++)
		for (int j = 0; j<3; j++)
			if (i == j) rt[i][j] = 1;
			else rt[i][j] = 0;
}

/////////////////////////////////////////////////////////////////////////////
//
//  @\     : RRsñÌæZ
//
//  üÍ     : r		RRsñ
//  @@       rt		RRsñ
//
//  Ôèl   : r <-	r  rt
//
//  õl     : ÄÑoµt@NV
//@		@ Èµ
//
/////////////////////////////////////////////////////////////////////////////

void rMatrixmulti(MATRIX &r, MATRIX &rt)
{
	double	rin[3][3];
	int		i, j;

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			rin[i][j] = r[i][j];

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++) {
			r[i][j] = rin[i][0] * rt[0][j] +
				rin[i][1] * rt[1][j] +
				rin[i][2] * rt[2][j];
		}
}

/////////////////////////////////////////////////////////////////////////////
//
//  @\     : ñ]pxÉæèRRÀWnÏ·sñðßé
//
//  üÍ     : rt		RRÀWnÏ·sñ
//  @@       rotateX	w²Ìñ]pi[pj   PÊF
//  @@       rotateY	x²Ìñ]pis[`pj
//  @@       rotateZ	y²Ìñ]pi[pj
//
//  Ôèl   : rt <- Rz * Ry * Rx
//
//  õl     : ÄÑoµt@NV
//@		@ @ rMatrixmulti
//
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_ZYX(MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			if (i == j) rt[i][j] = 1;
			else rt[i][j] = 0;

			if (rotateZ != 0.0) {
				/*	R3 :   cosz  -sinz   0.0
				sinz  cosz   0.0
				0.0   0.0   1.0
				*/
				rr[0][0] = cosz;
				rr[0][1] = -sinz;
				rr[0][2] = 0.0;
				rr[1][0] = sinz;
				rr[1][1] = cosz;
				rr[1][2] = 0.0;
				rr[2][0] = 0.0;
				rr[2][1] = 0.0;
				rr[2][2] = 1.0;
				rMatrixmulti(rt, rr);
			}

			if (rotateY != 0.0) {
				/*	R2 :   cosy   0.0  siny
				0.0   1.0   0.0
				-siny   0.0  cosy
				*/
				rr[0][0] = cosy;
				rr[0][1] = 0.0;
				rr[0][2] = siny;
				rr[1][0] = 0.0;
				rr[1][1] = 1.0;
				rr[1][2] = 0.0;
				rr[2][0] = -siny;
				rr[2][1] = 0.0;
				rr[2][2] = cosy;
				rMatrixmulti(rt, rr);
			}

			if (rotateX != 0.0) {
				/*	R1 :	1.0   0.0   0.0
				0.0  cosx  -sinx
				0.0  sinx  cosx
				*/
				rr[0][0] = 1.0;
				rr[0][1] = 0.0;
				rr[0][2] = 0.0;
				rr[1][0] = 0.0;
				rr[1][1] = cosx;
				rr[1][2] = -sinx;
				rr[2][0] = 0.0;
				rr[2][1] = sinx;
				rr[2][2] = cosx;
				rMatrixmulti(rt, rr);
			}
}

/////////////////////////////////////////////////////////////////////////////
//
//  @\     : ñ]pxÉæèRRÀWnÏ·sñðßé
//
//  üÍ     : rt		RRÀWnÏ·sñ
//  @@       rotateX	w²Ìñ]pi[pj   PÊF
//  @@       rotateY	x²Ìñ]pis[`pj
//  @@       rotateZ	y²Ìñ]pi[pj
//
//  Ôèl   : rt <- Rx * Ry * Rz
//
//  õl     : ÄÑoµt@NV
//@		@ @ rMatrixmulti
//
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_XYZ(MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			if (i == j) rt[i][j] = 1;
			else rt[i][j] = 0;

			if (rotateX != 0.0) {
				/*	R1 :	1.0   0.0   0.0
				0.0  cosx  -sinx
				0.0  sinx  cosx
				*/
				rr[0][0] = 1.0;
				rr[0][1] = 0.0;
				rr[0][2] = 0.0;
				rr[1][0] = 0.0;
				rr[1][1] = cosx;
				rr[1][2] = -sinx;
				rr[2][0] = 0.0;
				rr[2][1] = sinx;
				rr[2][2] = cosx;
				rMatrixmulti(rt, rr);
			}

			if (rotateY != 0.0) {
				/*	R2 :   cosy   0.0 siny
				0.0   1.0   0.0
				-siny   0.0  cosy
				*/
				rr[0][0] = cosy;
				rr[0][1] = 0.0;
				rr[0][2] = siny;
				rr[1][0] = 0.0;
				rr[1][1] = 1.0;
				rr[1][2] = 0.0;
				rr[2][0] = -siny;
				rr[2][1] = 0.0;
				rr[2][2] = cosy;
				rMatrixmulti(rt, rr);
			}

			if (rotateZ != 0.0) {
				/*	R3 :   cosz  -sinz   0.0
				sinz cosz   0.0
				0.0   0.0   1.0
				*/
				rr[0][0] = cosz;
				rr[0][1] = -sinz;
				rr[0][2] = 0.0;
				rr[1][0] = sinz;
				rr[1][1] = cosz;
				rr[1][2] = 0.0;
				rr[2][0] = 0.0;
				rr[2][1] = 0.0;
				rr[2][2] = 1.0;
				rMatrixmulti(rt, rr);
			}
}

//Mei Jilin 20160716
void createRotMatrix_XYZ_Inv(MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			if (i == j) rt[i][j] = 1;
			else rt[i][j] = 0;

			if (rotateX != 0.0) {
				/*	R1 :	1.0   0.0   0.0
				0.0  cosx  sinx
				0.0  -sinx  cosx
				*/
				rr[0][0] = 1.0;
				rr[0][1] = 0.0;
				rr[0][2] = 0.0;
				rr[1][0] = 0.0;
				rr[1][1] = cosx;
				rr[1][2] = sinx;
				rr[2][0] = 0.0;
				rr[2][1] = -sinx;
				rr[2][2] = cosx;
				rMatrixmulti(rt, rr);
			}

			if (rotateY != 0.0) {
				/*	R2 :   cosy   0.0  -siny
				0.0   1.0   0.0
				siny   0.0  cosy
				*/
				rr[0][0] = cosy;
				rr[0][1] = 0.0;
				rr[0][2] = -siny;
				rr[1][0] = 0.0;
				rr[1][1] = 1.0;
				rr[1][2] = 0.0;
				rr[2][0] = siny;
				rr[2][1] = 0.0;
				rr[2][2] = cosy;
				rMatrixmulti(rt, rr);
			}

			if (rotateZ != 0.0) {
				/*	R3 :   cosz  sinz   0.0
				-sinz cosz   0.0
				0.0   0.0   1.0
				*/
				rr[0][0] = cosz;
				rr[0][1] = sinz;
				rr[0][2] = 0.0;
				rr[1][0] = -sinz;
				rr[1][1] = cosz;
				rr[1][2] = 0.0;
				rr[2][0] = 0.0;
				rr[2][1] = 0.0;
				rr[2][2] = 1.0;
				rMatrixmulti(rt, rr);
			}
}

void GetRectPts(point2d *cp, point2d *v1, double olen1, double olen2, point2d *p, double margin)
{
	double	len1 = olen1 / 2 + margin;
	double	len2 = olen2 / 2 + margin;

	point2d	v2;
	rotatePoint90(v1, &v2);

	// 1,-1
	p[0].x = cp->x + v1->x*len1 - v2.x*len2;
	p[0].y = cp->y + v1->y*len1 - v2.y*len2;

	// 1,1
	p[1].x = cp->x + v1->x*len1 + v2.x*len2;
	p[1].y = cp->y + v1->y*len1 + v2.y*len2;

	// -1,1
	p[2].x = cp->x - v1->x*len1 + v2.x*len2;
	p[2].y = cp->y - v1->y*len1 + v2.y*len2;

	// -1,-1
	p[3].x = cp->x - v1->x*len1 - v2.x*len2;
	p[3].y = cp->y - v1->y*len1 - v2.y*len2;

}

void rotatePoint90(point2d *p, point2d *p90)
{
	p90->x = -p->y;
	p90->y = p->x;
}

void shiftPoint2d(point2d *pt, point2d *sh)
{
	point2d		p;

	p.x = pt->x + sh->x;
	p.y = pt->y + sh->y;
	pt->x = p.x;
	pt->y = p.y;
}

void rotatePoint2d(point2d *pt, double a[2][2])
{
	double x, y;

	x = a[0][0] * pt->x + a[0][1] * pt->y;
	y = a[1][0] * pt->x + a[1][1] * pt->y;
	pt->x = x;
	pt->y = y;
}

/////////////////////////////////////////////////////////////////////////////
//
//  @\     : R³|CgðVtg·é
//
//  üÍ     : pt		R³|Cg
//  @@@@@ sh		Ú®ÊxNg
//
//  Ôèl   : Èµ
//
//  õl     : ÄÑoµt@NV
//@		@ Èµ
//
/////////////////////////////////////////////////////////////////////////////

void shiftPoint3d(point3d &pt, point3d &sh)
{
	point3d		p;

	p.x = pt.x + sh.x;
	p.y = pt.y + sh.y;
	p.z = pt.z + sh.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

/////////////////////////////////////////////////////////////////////////////
//
//  @\     : R³|Cgðñ]·é
//
//  üÍ     : pt		R³|Cg
//  @@@@@ a		ñ]sñ
//
//  Ôèl   : Èµ
//
//  õl     : ÄÑoµt@NV
//@		@ Èµ
//
/////////////////////////////////////////////////////////////////////////////

void rotatePoint3d(point3d &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0] * pt.x + a[0][1] * pt.y + a[0][2] * pt.z;
	p.y = a[1][0] * pt.x + a[1][1] * pt.y + a[1][2] * pt.z;
	p.z = a[2][0] * pt.x + a[2][1] * pt.y + a[2][2] * pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void normalPoint3d(point3d &pt)
{
	double dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
	pt.x /= dist;
	pt.y /= dist;
	pt.z /= dist;
}

void zp2cvp(cv::Point3d& cvp, point3d &zp)
{
	cvp.x = zp.x;
	cvp.y = zp.y;
	cvp.z = zp.z;
}

void cvp2zp(cv::Point3d& cvp, point3d &zp)
{
	zp.x = cvp.x;
	zp.y = cvp.y;
	zp.z = cvp.z;
}

void shiftPoint3d(cv::Point3d & pt, point3d & sh)
{
	point3d _pt;
	point3d _sh = sh;
	cvp2zp(pt, _pt);
	shiftPoint3d(_pt, _sh);
	zp2cvp(pt, _pt);
}

void rotatePoint3d(cv::Point3d & pt, MATRIX & a)
{
	point3d _pt;
	cvp2zp(pt, _pt);
	rotatePoint3d(_pt, a);
	zp2cvp(pt, _pt);
}

void normalPoint3d(cv::Point3d & pt)
{
	point3d _pt;
	cvp2zp(pt, _pt);
	normalPoint3d(_pt);
	zp2cvp(pt, _pt);
}

void rotatePoint2d(cv::Point2d & pt, double angle)
{
	double _cos = cos(angle);
	double _sin = sin(angle);
	double x = _cos * pt.x - _sin * pt.y;
	double y = _sin * pt.x + _cos * pt.y;
	pt.x = x;
	pt.y = y;
}

////



void calcTrans(TRANSINFO& trans)
{
	createRotMatrix_XYZ(trans.rot, trans.ang.x, trans.ang.y, trans.ang.z);
	createRotMatrix_ZYX(trans.invrot, -trans.ang.x, -trans.ang.y, -trans.ang.z);
	trans.invshv.x = -trans.shv.x;
	trans.invshv.y = -trans.shv.y;
	trans.invshv.z = -trans.shv.z;
	return;
}

bool loadTrans(char* s, TRANSINFO& trans)
{
	sscanf(s, "%lf%lf%lf%lf%lf%lf", &trans.ang.x, &trans.ang.y, &trans.ang.z, &trans.shv.x, &trans.shv.y, &trans.shv.z);
	trans.ang.x = trans.ang.x * CV_PI / 180.0;//deg -> rad
	trans.ang.y = trans.ang.y * CV_PI / 180.0;
	trans.ang.z = trans.ang.z * CV_PI / 180.0;
	calcTrans(trans);
	return true;
}

bool CoordinateTrans::LoadCalib(std::string filename)
{
	if (filename == "")
	{
		printf("calib load error\n");
		return false;
	}

	std::ifstream calibfile;
	calibfile.open(filename.c_str());
	if (!calibfile.is_open())
	{
		printf("calib load error\n");
		return false;
	}

	std::string header;
	calibfile >> header >> trans.ang.x >> trans.ang.y >> trans.ang.z;
	calibfile >> header >> trans.shv.x >> trans.shv.y >> trans.shv.z;

	createRotMatrix_XYZ(trans.rot, trans.ang.x, trans.ang.y, trans.ang.z);
	createRotMatrix_ZYX(trans.invrot, -trans.ang.x, -trans.ang.y, -trans.ang.z);
	trans.invshv.x = -trans.shv.x;
	trans.invshv.y = -trans.shv.y;
	trans.invshv.z = -trans.shv.z;

	calibfile.close();
	return true;
}

void CoordinateTrans::VehicleP2LocalP(const cv::Point3d & in, cv::Point3d & out)
{
	point3d pt;
	pt.x = in.x;
	pt.y = in.y;
	pt.z = in.z;
	shiftPoint3d(pt, trans.invshv);
	rotatePoint3d(pt, trans.invrot);
	out.x = pt.x;
	out.y = pt.y;
	out.z = pt.z;
}

void CoordinateTrans::LocalP2VehicleP(const cv::Point3d & in, cv::Point3d & out)
{
	point3d pt;
	pt.x = in.x;
	pt.y = in.y;
	pt.z = in.z;
	rotatePoint3d(pt, trans.rot);
	shiftPoint3d(pt, trans.shv);
	out.x = pt.x;
	out.y = pt.y;
	out.z = pt.z;
}

