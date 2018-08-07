#pragma once

#define	SCANDATASIZE		1001
#define	M_PI			3.1415926535		//M_PI and PI should be integrated after development
#define DEG2RAD(x)		( (x)*M_PI/180.0 )
#define GETRANGLEANG(i,start,step)	((double)(i)*(step)+(start))


typedef double  MATRIX[3][3];

class point3d
{
public:
	double x, y, z;
	point3d() {}
	point3d(double x, double y, double z) :
		x(x), y(y), z(z) {}
};

struct rgbpoint3d
{
	double x, y, z;
	unsigned char r, g, b;
};

typedef struct {
	int x, y;
} pointdisp;

typedef	struct {
	double	x, y;
} point2d;

typedef struct {
	long			Millisecond;
	short			dat[SCANDATASIZE];
	//前两个变量顺序一定不能个改，因为LoadLmsDataFromFile直接从lms文件中顺序读取出来的
	pointdisp		pt[SCANDATASIZE];
	short			lab[SCANDATASIZE];
	//ONETRAJ			traj[SCANDATASIZE];
	int				trajnum;
	point2d			localpt[SCANDATASIZE];

}LMSDAT;

typedef struct {
	LMSDAT			*scandata;
	int				scannum;
}LMSDATBUF;

typedef struct {
	int				tno;
	pointdisp		p;
	pointdisp		pt[4];
	pointdisp       dpt[4];
	point2d			apt[4];
	point2d			adpt[4];
	point2d			v;
	point2d			gp;
	double          ang;
}ONETRAJ;

typedef struct {
	long		milli;
	ONETRAJ     traj[30];
	int			trajnum;
}TRAJ;

typedef struct {
	TRAJ		*trajs;
	int			num;
}TRAJS;

typedef struct {
	long	milli;
	int		fno;
	point2d gp;
	double	len0, len1;

	point2d gv1, gv2;
} TRAJBUF;

typedef struct {
	float			angrng;
	float			angres;
	float			unit;
	double			angStep;
	double			angStart;
	short			datnum;
}LMSHEADER;

typedef struct {
	float			angrng;
	float			angres;
	float			unit;
	double			angStep;
	double			angStart;
	short			datnum;
}DSHEADER;

typedef struct {
	point3d         ang;
	point3d         shv;
	long			Millisecond;
	short			dat[SCANDATASIZE];
	//point2d         lpt[SCANDATASIZE];
	short			lab[SCANDATASIZE];
	pointdisp		pt[SCANDATASIZE];
	//ONETRAJ			traj[SCANDATASIZE];
	int				trajnum;
}DSDAT;

typedef struct {
	DSDAT			*scandata;
	int				scannum;
}DSDATBUF;