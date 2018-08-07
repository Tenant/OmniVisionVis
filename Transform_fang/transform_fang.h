#ifndef COORDINATETRANSFORM_H
#define COORDINATETRANSFORM_H

#include "../define_zhao.h"
#include "../header.h"
#include <fstream>

//void rMatrixInit (cv::Mat &rt);
//void rMatrixmulti (cv::Mat &r, const cv::Mat &rt);
//void createRotMatrix_ZYX (cv::Mat &rt, double rotateX, double rotateY, double rotateZ);
//void createRotMatrix_XYZ (cv::Mat &rt, double rotateX, double rotateY, double rotateZ);
//void createRotMatrix_XYZ_Inv (cv::Mat &rt, double rotateX, double rotateY, double rotateZ);
//void shiftPoint3d (cv::Point3d &pt, cv::Point3d &sh);
//void rotatePoint3d (cv::Point3d &pt, cv::Mat &a);
//void normalPoint3d(cv::Point3d &pt);

//struct TRANSINFO{
//    char			name[20];
//    point3d			ang;
//    point3d			shv;
//    MATRIX			rot;
//    MATRIX			invrot;
//    point3d			invshv;
//    point3d			ang2;
//    point3d			shv2;
//    MATRIX			rot2;
//    MATRIX			invrot2;
//    point3d			invshv2;
//    bool			reverse;
//};

//calculation
void rMatrixInit(MATRIX &rt);
void rMatrixmulti(MATRIX &r, MATRIX &rt);
void createRotMatrix_ZYX(MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void createRotMatrix_XYZ(MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void createRotMatrix_XYZ_Inv(MATRIX &rt, double rotateX, double rotateY, double rotateZ);

void rotatePoint90(point2d *p, point2d *p90);
void GetRectPts(point2d *cp, point2d *v1, double olen1, double olen2, point2d *p, double margin);
void shiftPoint2d(point2d *pt, point2d *sh);
void rotatePoint2d(point2d *pt, double a[2][2]);

void shiftPoint3d(point3d &pt, point3d &sh);
void rotatePoint3d(point3d &pt, MATRIX &a);
void normalPoint3d(point3d &pt);

void zp2cvp(cv::Point3d& cvp, point3d &zp);
void cvp2zp(cv::Point3d& cvp, point3d &zp);
void shiftPoint3d(cv::Point3d &pt, point3d &sh);
void rotatePoint3d(cv::Point3d &pt, MATRIX &a);
void normalPoint3d(cv::Point3d &pt);
void rotatePoint2d(cv::Point2d& pt, double angle);

///
struct TRANSINFO {
	char			name[20];
	point3d			ang;
	point3d			shv;
	MATRIX			rot;
	MATRIX			invrot;
	point3d			invshv;
	bool			valid;
};

void calcTrans(TRANSINFO& trans);
bool loadTrans(char* s, TRANSINFO& trans);

class CoordinateTrans
{
public:
	bool LoadCalib(std::string filename);
	void LocalP2VehicleP(const cv::Point3d& in, cv::Point3d& out);
	void VehicleP2LocalP(const cv::Point3d& in, cv::Point3d& out);
private:
	TRANSINFO trans;
};

#endif // COORDINATETRANSFORM_H
