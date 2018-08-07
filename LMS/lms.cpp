#include "lms.h"

LMS::LMS()
{
	lmsTrans.ang.x = 0;
	lmsTrans.ang.y = 0;
	lmsTrans.ang.z = 0;
	lmsTrans.shv.x = 0;
	lmsTrans.shv.y = 0;
	lmsTrans.shv.z = 0;
	lmsTrans.ang.x = DEG2RAD(lmsTrans.ang.x);
	lmsTrans.ang.y = DEG2RAD(lmsTrans.ang.y);
	lmsTrans.ang.z = DEG2RAD(lmsTrans.ang.z);
	CalcTrans(lmsTrans);
	cameraTrans.ang.x = 0;
	cameraTrans.ang.y = 0;
	cameraTrans.ang.z = 119.9+70.0;//有可能每次做实验安装的时候有一个旋转
	cameraTrans.ang.x = DEG2RAD(cameraTrans.ang.x);
	cameraTrans.ang.y = DEG2RAD(cameraTrans.ang.y);
	cameraTrans.ang.z = DEG2RAD(cameraTrans.ang.z);
 	cameraTrans.shv.x = 0.4;// 日立文档
 	cameraTrans.shv.y = 0;
 	cameraTrans.shv.z = 1.0;
	//cameraTrans.shv.x = 0.4;
	//cameraTrans.shv.y = -2.6;
	//cameraTrans.shv.z = 1.3;
	//cameraTrans.shv.x = 0.31564;
	//cameraTrans.shv.y = 4.6-2.86566;
	//cameraTrans.shv.z = 1.55768;
	CalcTrans(cameraTrans);
	trajs.num=0;
	trajs.trajs = (TRAJ *)malloc(sizeof(TRAJ)*20000);//所以最多20000个traj
	/*cameraTrans.rot[0][0] = -0.157745;
	cameraTrans.rot[0][1] = -0.987421;
	cameraTrans.rot[0][2] = 0.0107767;
	cameraTrans.rot[1][0] = 0.987415;
	cameraTrans.rot[1][1] = -0.157851;
	cameraTrans.rot[1][2] = -0.0097472;
	cameraTrans.rot[2][0] = 0.0113257;
	cameraTrans.rot[2][1] = 0.00910347;
	cameraTrans.rot[2][2] = 0.999894;
	
	cameraTrans.invrot[0][0] = -0.157745586685147;
	cameraTrans.invrot[0][1] = 0.987414765699518;
	cameraTrans.invrot[0][2] = 0.011325706593155;
	cameraTrans.invrot[1][0] = -0.987421054835491;
	cameraTrans.invrot[1][1] = -0.157850384969823;
	cameraTrans.invrot[1][2] = 0.009103506180923;
	cameraTrans.invrot[2][0] = 0.010776679469207;
	cameraTrans.invrot[2][1] = -0.009747210372121;
	cameraTrans.invrot[2][2] = 0.999894843802868;

	cameraTrans.invshv.x = -cameraTrans.shv.x;
	cameraTrans.invshv.y = -cameraTrans.shv.y;
	cameraTrans.invshv.z = -cameraTrans.shv.z;

	*/
	

	for(int i=0;i<20000;i++){
		trajs.trajs[i].trajnum = 0;
		trajs.trajs[i].milli = 0;
	}
	trajs.trajs[0].milli = 59400199;//我不太明白这个是啥意思...
	trajs.trajs[1].milli = 59400251;

}

LMS::~LMS()
{
	free(trajs.trajs);
}

void LMS::CalcTrans(TRANSINFO& trans)
{
	createRotMatrix_ZYX(trans.rot,		trans.ang.x, trans.ang.y, trans.ang.z);
	createRotMatrix_XYZ(trans.invrot,  -trans.ang.x,-trans.ang.y,-trans.ang.z);
	trans.invshv.x = -trans.shv.x;
	trans.invshv.y = -trans.shv.y;
	trans.invshv.z = -trans.shv.z;
	return;
}

void LMS::initTrans()
{
	lmsTrans.ang.x = lmsTrans.ang.y = lmsTrans.ang.z = 0;
	lmsTrans.shv.x = lmsTrans.shv.y = lmsTrans.shv.z = 0;
	CalcTrans(lmsTrans);
	cameraTrans.ang.x = cameraTrans.ang.y = cameraTrans.ang.z = 0;
	CalcTrans(cameraTrans);
	return;
}

bool LMS::LoadLmsDataFromFile(const char *fn)
{
	HFILE fp;
	OFSTRUCT of;
	DWORD memnum;
	UINT count;
	int i,j;
	int datasize;

	size_t namelen = strlen(fn);
	char filename[200];
	strcpy(filename, fn);
	i = 1;
	//for (i=1;i<5;i++)
	{
		if(i==1)
			filename[namelen-1]='1';
		if(i==2)
			filename[namelen-1]='2';
		if(i==3)
			filename[namelen-1]='3';
		if(i==4)
			filename[namelen-1]='4';
	
		fp = OpenFile(filename, &of, (UINT)OF_READ);
		if(fp == -1)
			return false;
		_lread(fp,&lmsHeader[i-1],sizeof(float)*3);

		lmsHeader[i-1].angStart = 0;
		lmsHeader[i-1].angStep = DEG2RAD(lmsHeader[i-1].angres);
		/*
		if (lmsHeader.angres==0.5 && lmsHeader.angrng==180)
		lmsHeader.datnum = 361;
		if (lmsHeader.angres==1 && lmsHeader.angrng==180)
		lmsHeader.datnum = 181;
		*/
		lmsHeader[i-1].datnum = (int)(lmsHeader[i-1].angrng/lmsHeader[i-1].angres+1);

		datasize = sizeof(long)+sizeof(short)*lmsHeader[i-1].datnum;
		memnum = _llseek( fp, 0L, SEEK_END );
		lmsDatBuf[i-1].scannum = memnum / datasize;
		memnum = lmsDatBuf[i-1].scannum * sizeof(LMSDAT);
		lmsDatBuf[i-1].scandata = (LMSDAT *)malloc (memnum);
		memset(lmsDatBuf[i-1].scandata,0,memnum);

		_llseek(fp, sizeof(float)*3, SEEK_SET);
		_lread(fp, &lmsDatBuf[i-1].scandata[0], datasize);

		for(j=0; j<lmsDatBuf[i-1].scannum-1; j++){
			count = _lread(fp, &lmsDatBuf[i-1].scandata[j], datasize);
			if(!count)
				break;
			LmsptOnPgr(i,j,1024,2048);
		//	lmsDatBuf.scandata[i].trajnum = 0;
		}
		_lclose(fp);
	}
	return true;

}

bool LMS::LoadDsDataFromFile(char *filename)
{
	HFILE fp;
	OFSTRUCT of;
	DWORD memnum;
	UINT count;
	int i;
	int datasize;

	fp = OpenFile(filename, &of, (UINT)OF_READ);
	if(fp == -1)
		return false;
	_lread(fp,&dsHeader,sizeof(float)*3);
	dsHeader.angStart = 0;
	dsHeader.angStep = DEG2RAD(dsHeader.angres);
	/*
	if (lmsHeader.angres==0.5 && lmsHeader.angrng==180)
	lmsHeader.datnum = 361;
	if (lmsHeader.angres==1 && lmsHeader.angrng==180)
	lmsHeader.datnum = 181;
	*/
	dsHeader.datnum = (int)(dsHeader.angrng/dsHeader.angres+1);
	datasize = sizeof(point3d)*2+sizeof(long)+sizeof(short)*dsHeader.datnum;
	memnum = _llseek( fp, 0L, SEEK_END );
	dsDatBuf.scannum = memnum / datasize;
	memnum = dsDatBuf.scannum * sizeof(DSDAT);
	dsDatBuf.scandata = (DSDAT *)malloc (memnum);
	memset(dsDatBuf.scandata,0,memnum);

	_llseek(fp, sizeof(float)*3, SEEK_SET);
	//_lread(fp, &dsDatBuf.scandata[0], datasize);

	for(i=0; i<dsDatBuf.scannum; i++){
		count = _lread(fp, &dsDatBuf.scandata[i], datasize);
		if(!count)
			break;
		DsptOnPgr(i,1024,2048);
	//	dsDatBuf.scandata[i].trajnum = 0;
	}
	_lclose(fp);
	
	size_t namelen = strlen(filename);
	filename[namelen-1] = '2';

	fp = OpenFile(filename, &of, (UINT)OF_READ);
	if(fp == -1)
		return false;
	_lread(fp,&dsHeader1,sizeof(float)*3);
	dsHeader1.angStart = 0;
	dsHeader1.angStep = DEG2RAD(dsHeader1.angres);
	/*
	if (lmsHeader.angres==0.5 && lmsHeader.angrng==180)
	lmsHeader.datnum = 361;
	if (lmsHeader.angres==1 && lmsHeader.angrng==180)
	lmsHeader.datnum = 181;
	*/
	dsHeader1.datnum = (int)(dsHeader1.angrng/dsHeader1.angres+1);
	datasize = sizeof(point3d)*2+sizeof(long)+sizeof(short)*dsHeader1.datnum;
	memnum = _llseek( fp, 0L, SEEK_END );
	dsDatBuf1.scannum = memnum / datasize;
	memnum = dsDatBuf1.scannum * sizeof(DSDAT);
	dsDatBuf1.scandata = (DSDAT *)malloc (memnum);
	memset(dsDatBuf1.scandata,0,memnum);

	_llseek(fp, sizeof(float)*3, SEEK_SET);
	//_lread(fp, &dsDatBuf.scandata[0], datasize);

	for(i=0; i<dsDatBuf1.scannum; i++){
		count = _lread(fp, &dsDatBuf1.scandata[i], datasize);
		if(!count)
			break;
		DsptOnPgr1(i,1024,2048);
		//	dsDatBuf.scandata[i].trajnum = 0;
	}
	_lclose(fp);


	filename[namelen-1] = '3';

	fp = OpenFile(filename, &of, (UINT)OF_READ);
	if(fp == -1)
		return false;
	_lread(fp,&dsHeader2,sizeof(float)*3);
	dsHeader2.angStart =0;
	dsHeader2.angStep = DEG2RAD(dsHeader2.angres);
	/*
	if (lmsHeader.angres==0.5 && lmsHeader.angrng==180)
	lmsHeader.datnum = 361;
	if (lmsHeader.angres==1 && lmsHeader.angrng==180)
	lmsHeader.datnum = 181;
	*/
	dsHeader2.datnum = (int)(dsHeader2.angrng/dsHeader2.angres+1);
	datasize = sizeof(point3d)*2+sizeof(long)+sizeof(short)*dsHeader2.datnum;
	memnum = _llseek( fp, 0L, SEEK_END );
	dsDatBuf2.scannum = memnum / datasize;
	memnum = dsDatBuf2.scannum * sizeof(DSDAT);
	dsDatBuf2.scandata = (DSDAT *)malloc (memnum);
	memset(dsDatBuf2.scandata,0,memnum);

	_llseek(fp, sizeof(float)*3, SEEK_SET);
	//_lread(fp, &dsDatBuf.scandata[0], datasize);

	for(i=0; i<dsDatBuf2.scannum; i++){
		count = _lread(fp, &dsDatBuf2.scandata[i], datasize);
		if(!count)
			break;
		DsptOnPgr2(i,1024,2048);
		//	dsDatBuf.scandata[i].trajnum = 0;
	}
	_lclose(fp);

	filename[namelen-1] = '4';

	fp = OpenFile(filename, &of, (UINT)OF_READ);
	if(fp == -1)
		return false;
	_lread(fp,&dsHeader3,sizeof(float)*3);
	dsHeader3.angStart =0;
	dsHeader3.angStep = DEG2RAD(dsHeader3.angres);
	/*
	if (lmsHeader.angres==0.5 && lmsHeader.angrng==180)
	lmsHeader.datnum = 361;
	if (lmsHeader.angres==1 && lmsHeader.angrng==180)
	lmsHeader.datnum = 181;
	*/
	dsHeader3.datnum = (int)(dsHeader3.angrng/dsHeader3.angres+1);
	datasize = sizeof(point3d)*2+sizeof(long)+sizeof(short)*dsHeader3.datnum;
	memnum = _llseek( fp, 0L, SEEK_END );
	dsDatBuf3.scannum = memnum / datasize;
	memnum = dsDatBuf3.scannum * sizeof(DSDAT);
	dsDatBuf3.scandata = (DSDAT *)malloc (memnum);
	memset(dsDatBuf3.scandata,0,memnum);

	_llseek(fp, sizeof(float)*3, SEEK_SET);
	//_lread(fp, &dsDatBuf.scandata[0], datasize);

	for(i=0; i<dsDatBuf3.scannum; i++){
		count = _lread(fp, &dsDatBuf3.scandata[i], datasize);
		if(!count)
			break;
		DsptOnPgr3(i,1024,2048);
		//	dsDatBuf.scandata[i].trajnum = 0;
	}
	_lclose(fp);


	return true;

}

void LMS::LmsptOnPgr(int ldno,int fno, int row, int col)
{
	int				dno, x, y;
	double			dDistance, dAngle;
	point3d			pL;
	double			angle,longitude, latitude;
	double rot[2][2];
	point2d	shv;
	point2d p;
	bool reverseFlag = false;
//这里其实没必要严格地把标定参数调对，毕竟激光和视频时间并不是严格一一对应，高速情况下是可能有较大偏差的
/*	if(ldno==1){
		shv.x=0.2;//老实说觉得这个标定参数很奇怪
		shv.y=2.4;
		angle=-5;				
	}
	if(ldno==2){
		shv.x=-0.58;
		shv.y=1.94;
		angle=22.6;				
	}
	if(ldno==3){
		shv.x=1.08;
		shv.y=1.92;
		angle=-106.0;				
	}
	if(ldno==4){
		shv.x=0.4;
		shv.y=-2.4;
		angle=180;				
	}*/
	if (ldno == 1){//这个是标定到GPS的位置（惯性导航模块的中心，不是车体中心）
		reverseFlag = true;
		shv.x = 0.28;
		shv.y = 2.6;
		angle = -5.8;//老实说觉得这个标定参数很奇怪，文档里面事故后是4.2，但是修改为这个之后才能和其他的激光匹配上  ; 因为后来的激光是190°采的，而且又reverse了一下。
		/*shv.x=0;
		shv.y=0;
		angle=0;*/
	}
	if (ldno == 2){
		shv.x = -0.46;
		shv.y = 2.18;
		angle = 23;
	}
	if (ldno == 3){
		shv.x = 1.08;
		shv.y = 2.26;
		angle = -106.6;
	}
	if (ldno == 4){
		shv.x = 0.38;
		shv.y = -2.24;
		angle = 176.6;
	}

	rot[0][0] = cos(angle*M_PI/180.0);
	rot[0][1] = -sin(angle*M_PI/180.0);
	rot[1][0] = sin(angle*M_PI/180.0);
	rot[1][1] = cos(angle*M_PI/180.0);
	
	for(dno=0;dno<lmsHeader[ldno-1].datnum;dno++){
		lmsDatBuf[ldno-1].scandata[fno].lab[dno] = -1;
		if (reverseFlag)
		{
			dAngle = GETRANGLEANG(lmsHeader[ldno - 1].datnum - 1 - dno, lmsHeader[ldno - 1].angStart, lmsHeader[ldno - 1].angStep);
		}
		else
		{
			dAngle = GETRANGLEANG(dno, lmsHeader[ldno - 1].angStart, lmsHeader[ldno - 1].angStep);
		}
		dDistance = (double)lmsDatBuf[ldno-1].scandata[fno].dat[dno] / lmsHeader[ldno-1].unit;
		p.x = dDistance*cos(dAngle);
		p.y = dDistance*sin(dAngle);
		rotatePoint2d(&p,rot);
		shiftPoint2d(&p,&shv);
		pL.x = p.x;
		pL.y = p.y;
		pL.z = -0.7;//=0才是激光原始数据的高度。-0.7才是地面2016年1月11日20:26:03// -0.35;// -0.7就是地面 但是0并不是激光的高度，-0.35差不多吧
		// 转换到车体坐标，先旋转再平移
		rotatePoint3d(pL,lmsTrans.rot);
		shiftPoint3d(pL,lmsTrans.shv);

		lmsDatBuf[ldno - 1].scandata[fno].localpt[dno].x = pL.x;
		lmsDatBuf[ldno - 1].scandata[fno].localpt[dno].y = pL.y;

		// 转换到相机坐标，先平移再旋转
		shiftPoint3d(pL,cameraTrans.invshv);
		rotatePoint3d(pL,cameraTrans.invrot);

		normalPoint3d(pL);
		if (pL.y>=0)
			longitude = acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		else
			longitude = -acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		latitude = asin(pL.z);

		longitude = (1 - longitude/M_PI) /2 ;
		latitude = 0.5 - latitude/M_PI;
		x = int(longitude*col);
		y = int(latitude*row);
		if (x<0)	x = 0;
		if (x>=col)	x = col-1;
		if (y<0)	y = 0;
		if (y>=row)	y = row-1;

		lmsDatBuf[ldno-1].scandata[fno].pt[dno].x = x;
		lmsDatBuf[ldno-1].scandata[fno].pt[dno].y = y;
	}
}

void LMS::DsptOnPgr(int fno, int row, int col)
{
	int				dno, x, y;
	double			dDistance, dAngle;
	point3d			pL;
	double			longitude, latitude;
	//double rot[2][2];
	point2d	shv;
	point2d p;
	shv.x=0.2;
	shv.y=2.4;
	for(dno=0;dno<dsHeader.datnum;dno++){
		dsDatBuf.scandata[fno].lab[dno] = -1;
		dAngle = GETRANGLEANG(dno, dsHeader.angStart, dsHeader.angStep);
		dDistance = (double)dsDatBuf.scandata[fno].dat[dno] / dsHeader.unit;
		p.x = dDistance*cos(dAngle);
		p.y = dDistance*sin(dAngle);
		shiftPoint2d(&p,&shv);
		pL.x = p.x;
		pL.y = p.y;
		pL.z = 0;
	//	dsDatBuf.scandata[fno].lpt[dno].x = p.x;
	//	dsDatBuf.scandata[fno].lpt[dno].y = p.y;
		// 转换到车体坐标，先旋转再平移
		rotatePoint3d(pL,lmsTrans.rot);
		shiftPoint3d(pL,lmsTrans.shv);
		// 转换到相机坐标，先平移再旋转
		shiftPoint3d(pL,cameraTrans.invshv);
		rotatePoint3d(pL,cameraTrans.invrot);

		normalPoint3d(pL);
		if (pL.y>=0)
			longitude = acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		else
			longitude = -acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		latitude = asin(pL.z);

		longitude = (1 - longitude/M_PI) /2 ;
		latitude = 0.5 - latitude/M_PI;
		x = int(longitude*col);
		y = int(latitude*row);
		if (x<0)	x = 0;
		if (x>=col)	x = col-1;
		if (y<0)	y = 0;
		if (y>=row)	y = row-1;

		dsDatBuf.scandata[fno].pt[dno].x = x;
		dsDatBuf.scandata[fno].pt[dno].y = y;
	}
}

void LMS::DsptOnPgr1(int fno, int row, int col)
{
	int				dno, x, y;
	double			dDistance, dAngle;
	point3d			pL;
	double			longitude, latitude;
	double rot[2][2];
	point2d	shv;
	point2d p;
	shv.x=-0.58;
	shv.y=1.94;
	rot[0][0] = cos(22.6*M_PI/180.0);
	rot[0][1] = -sin(22.6*M_PI/180.0);
	rot[1][0] = sin(22.6*M_PI/180.0);
	rot[1][1] = cos(22.6*M_PI/180.0);

	for(dno=0;dno<dsHeader1.datnum;dno++){
		dsDatBuf1.scandata[fno].lab[dno] = -1;
		dAngle = GETRANGLEANG(dno, dsHeader1.angStart, dsHeader1.angStep);
		dDistance = (double)dsDatBuf1.scandata[fno].dat[dno] / dsHeader1.unit;
		p.x = dDistance*cos(dAngle);
		p.y = dDistance*sin(dAngle);
		
		rotatePoint2d(&p,rot);
		shiftPoint2d(&p,&shv);
		pL.x = p.x;
		pL.y = p.y;
		pL.z = 0;
		//dsDatBuf1.scandata[fno].lpt[dno].x = p.x;
		//dsDatBuf1.scandata[fno].lpt[dno].y = p.y;
		// 转换到车体坐标，先旋转再平移
		rotatePoint3d(pL,lmsTrans.rot);
		shiftPoint3d(pL,lmsTrans.shv);
		// 转换到相机坐标，先平移再旋转
		shiftPoint3d(pL,cameraTrans.invshv);
		rotatePoint3d(pL,cameraTrans.invrot);

		normalPoint3d(pL);
		if (pL.y>=0)
			longitude = acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		else
			longitude = -acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		latitude = asin(pL.z);

		longitude = (1 - longitude/M_PI) /2 ;
		latitude = 0.5 - latitude/M_PI;
		x = int(longitude*col);
		y = int(latitude*row);
		if (x<0)	x = 0;
		if (x>=col)	x = col-1;
		if (y<0)	y = 0;
		if (y>=row)	y = row-1;

		dsDatBuf1.scandata[fno].pt[dno].x = x;
		dsDatBuf1.scandata[fno].pt[dno].y = y;
	}
}

void LMS::DsptOnPgr2(int fno, int row, int col)
{
	int				dno, x, y;
	double			dDistance, dAngle;
	point3d			pL;
	double			longitude, latitude;

	double rot[2][2];
	point2d	shv;
	point2d p;
	shv.x=1.08;
	shv.y=1.92;
	rot[0][0] = cos(-106.0*M_PI/180.0);
	rot[0][1] = -sin(-106.0*M_PI/180.0);
	rot[1][0] = sin(-106.0*M_PI/180.0);
	rot[1][1] = cos(-106.0*M_PI/180.0);
	for(dno=0;dno<dsHeader2.datnum;dno++){
		dsDatBuf2.scandata[fno].lab[dno] = -1;
		dAngle = GETRANGLEANG(dno, dsHeader2.angStart, dsHeader2.angStep);
		dDistance = (double)dsDatBuf2.scandata[fno].dat[dno] / dsHeader2.unit;
		p.x = dDistance*cos(dAngle);
		p.y = dDistance*sin(dAngle);
		
		rotatePoint2d(&p,rot);
		shiftPoint2d(&p,&shv);
		pL.x = p.x;
		pL.y = p.y;
		pL.z = 0;
		//dsDatBuf2.scandata[fno].lpt[dno].x = p.x;
		//dsDatBuf2.scandata[fno].lpt[dno].y = p.y;
		// 转换到车体坐标，先旋转再平移
		rotatePoint3d(pL,lmsTrans.rot);
		shiftPoint3d(pL,lmsTrans.shv);
		// 转换到相机坐标，先平移再旋转
		shiftPoint3d(pL,cameraTrans.invshv);
		rotatePoint3d(pL,cameraTrans.invrot);

		normalPoint3d(pL);
		if (pL.y>=0)
			longitude = acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		else
			longitude = -acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		latitude = asin(pL.z);

		longitude = (1 - longitude/M_PI) /2 ;
		latitude = 0.5 - latitude/M_PI;
		x = int(longitude*col);
		y = int(latitude*row);
		if (x<0)	x = 0;
		if (x>=col)	x = col-1;
		if (y<0)	y = 0;
		if (y>=row)	y = row-1;

		dsDatBuf2.scandata[fno].pt[dno].x = x;
		dsDatBuf2.scandata[fno].pt[dno].y = y;
	}
}

void LMS::DsptOnPgr3(int fno, int row, int col)
{
	int				dno, x, y;
	double			dDistance, dAngle;
	point3d			pL;
	double			longitude, latitude;
	double rot[2][2];
	point2d	shv;
	point2d p;
	shv.x=0.4;
	shv.y=-2.4;
	rot[0][0] = cos(180*M_PI/180.0);
	rot[0][1] = -sin(180*M_PI/180.0);
	rot[1][0] = sin(180*M_PI/180.0);
	rot[1][1] = cos(180*M_PI/180.0);

	for(dno=0;dno<dsHeader3.datnum;dno++){
		dsDatBuf3.scandata[fno].lab[dno] = -1;
		dAngle = GETRANGLEANG(dno, dsHeader3.angStart, dsHeader3.angStep);
		dDistance = (double)dsDatBuf3.scandata[fno].dat[dno] / dsHeader3.unit;
		p.x = dDistance*cos(dAngle);
		p.y = dDistance*sin(dAngle);
		
		rotatePoint2d(&p,rot);
		shiftPoint2d(&p,&shv);
		pL.x = p.x;
		pL.y = p.y;
		pL.z = 0;
		//dsDatBuf3.scandata[fno].lpt[dno].x = p.x;
		//dsDatBuf3.scandata[fno].lpt[dno].y = p.y;
		// 转换到车体坐标，先旋转再平移
		rotatePoint3d(pL,lmsTrans.rot);
		shiftPoint3d(pL,lmsTrans.shv);
		// 转换到相机坐标，先平移再旋转
		shiftPoint3d(pL,cameraTrans.invshv);
		rotatePoint3d(pL,cameraTrans.invrot);

		normalPoint3d(pL);
		if (pL.y>=0)
			longitude = acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		else
			longitude = -acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
		latitude = asin(pL.z);

		longitude = (1 - longitude/M_PI) /2 ;
		latitude = 0.5 - latitude/M_PI;
		x = int(longitude*col);
		y = int(latitude*row);
		if (x<0)	x = 0;
		if (x>=col)	x = col-1;
		if (y<0)	y = 0;
		if (y>=row)	y = row-1;

		dsDatBuf3.scandata[fno].pt[dno].x = x;
		dsDatBuf3.scandata[fno].pt[dno].y = y;
	}
}

bool LMS::FindFno(int ldno,long millisec, int& index)
{
	index = 0;
	long starttime = lmsDatBuf[ldno-1].scandata[0].Millisecond;
	long endtime = lmsDatBuf[ldno-1].scandata[lmsDatBuf[ldno-1].scannum-2].Millisecond;
	if(millisec < starttime){
		//qDebug("Pgr is earlier than Lms!!");
		return true;
	}
	if(millisec > endtime){
		//qDebug("Pgr is later than Lms!!");
		return false;
	}
	int tmpfno=0;
	//tmpfno = (millisec - starttime)/(lmsDatBuf.scandata[1].Millisecond-starttime);
	for(int i=0;i<lmsDatBuf[ldno-1].scannum-1;i++){
		if (millisec >= lmsDatBuf[ldno - 1].scandata[i].Millisecond && millisec < lmsDatBuf[ldno - 1].scandata[i + 1].Millisecond)
		{
			index = i;
			return true;
		}
	}
	return false;
}

int LMS::FindDsFno(long millisec)
{
	long starttime = dsDatBuf.scandata[0].Millisecond;
	long endtime = dsDatBuf.scandata[dsDatBuf.scannum-2].Millisecond;
	if(millisec < starttime){
		//qDebug("Pgr is earlier than Lms!!");
		return -1;
	}
	if(millisec > endtime){
		//qDebug("Pgr is later than Lms!!");
		return -1;
	}
	int tmpfno=0;
	//tmpfno = (millisec - starttime)/(lmsDatBuf.scandata[1].Millisecond-starttime);
	for(int i=0;i<dsDatBuf.scannum-1;i++){
		if(millisec >= dsDatBuf.scandata[i].Millisecond && millisec < dsDatBuf.scandata[i+1].Millisecond)
			return i;
	}
	return -1;


	/*
	while(1){
		if(millisec >= dsDatBuf.scandata[tmpfno].Millisecond &&millisec < dsDatBuf.scandata[tmpfno+1].Millisecond)
			break;
		else if(millisec < dsDatBuf.scandata[tmpfno].Millisecond)
			tmpfno --;
		else if(millisec >= dsDatBuf.scandata[tmpfno+1].Millisecond)
			tmpfno ++;
	}
	if((millisec-dsDatBuf.scandata[tmpfno].Millisecond)<(dsDatBuf.scandata[tmpfno+1].Millisecond-millisec) )
		return tmpfno;
	else
		return tmpfno+1;
		*/
}

point2d LMS::getLocalpt(int ldno, int fno, int dno)
{
	return lmsDatBuf[ldno - 1].scandata[fno].localpt[dno];
}

pointdisp LMS::getPgrpt(int ldno,int fno, int dno)
{
	return lmsDatBuf[ldno-1].scandata[fno].pt[dno];
}

double LMS::getDistance(int ldno,int fno, int dno)
{
	return (double)lmsDatBuf[ldno - 1].scandata[fno].dat[dno] / lmsHeader[ldno - 1].unit;
}

pointdisp LMS::getDsPgrpt(int fno, int dno)
{
	return dsDatBuf.scandata[fno].pt[dno];
}
pointdisp LMS::getDsPgrpt1(int fno, int dno)
{
	return dsDatBuf1.scandata[fno].pt[dno];
}
pointdisp LMS::getDsPgrpt2(int fno, int dno)
{
	return dsDatBuf2.scandata[fno].pt[dno];
}
pointdisp LMS::getDsPgrpt3(int fno, int dno)
{
	return dsDatBuf3.scandata[fno].pt[dno];
}

double LMS::getDsDistance(int fno, int dno)
{
	return (double)dsDatBuf.scandata[fno].dat[dno]/100.0;
}
double LMS::getDsDistance1(int fno, int dno)
{
	return (double)dsDatBuf1.scandata[fno].dat[dno]/100.0;
}
double LMS::getDsDistance2(int fno, int dno)
{
	return (double)dsDatBuf2.scandata[fno].dat[dno]/100.0;
}
double LMS::getDsDistance3(int fno, int dno)
{
	return (double)dsDatBuf3.scandata[fno].dat[dno]/100.0;
}

double LMS::getDsV(int fno)
{
	
	return dsDatBuf.scandata[fno].ang.z;
	
	
}

point2d LMS::getDsP(int fno)
{
	point2d v;
	v.x = dsDatBuf.scandata[fno].shv.x;
	v.y = dsDatBuf.scandata[fno].shv.y;
	return v;
}

int LMS::getLabel(int ldno,int fno, int dno)
{
	return lmsDatBuf[ldno-1].scandata[fno].lab[dno];
}

long LMS::getCurrentTime(int ldno, int index)
{
	return lmsDatBuf[ldno - 1].scandata[index + 1].Millisecond;
}

long LMS::getStartTime(int ldno)
{
	return lmsDatBuf[0].scandata[0].Millisecond;
}

long LMS::getEndTime(int ldno)
{
	return lmsDatBuf[0].scandata[lmsDatBuf[0].scannum-1].Millisecond;
}

int LMS::getDatNum(int ldno)
{
	return lmsHeader[ldno-1].datnum;
}

long LMS::getDsStartTime()
{
	return dsDatBuf.scandata[0].Millisecond;
}

long LMS::getDsEndTime()
{
	return dsDatBuf.scandata[dsDatBuf.scannum-1].Millisecond;
}

int LMS::getDsDatNum()
{
	return dsHeader.datnum;
}
int LMS::getDsDatNum1()
{
	return dsHeader1.datnum;
}

int LMS::getDsDatNum2()
{
	return dsHeader2.datnum;
}

int LMS::getDsDatNum3()
{
	return dsHeader3.datnum;
}

bool LMS::LoadTrajFile(char *filename)
{
	FILE	*fp;
	char	i_line[200];
	int		i,x,y,x2,y2;
	char	tokentno[] = "tno";
	char	tokenmilli[] = "milli";
	TRAJBUF	traj;
	int		nowtno = 0;
	int		tmpfno;
	point2d c[4],pp;
	point3d pL,dpL,p;
	double rot[2][2];
	point2d pt[4];
	point2d	shv;
	double			longitude, latitude,long2,lati2;
	fopen_s(&fp, filename, "r");
	//FILE *fp2;
	//fp2 = fopen("file.txt","wb");
	if (!fp) {
		fprintf (stderr, "Open data failure : %s", filename);
		return (FALSE);
	}
	fgets (i_line, 200, fp);
	do {
		if(_strnicmp(i_line,tokenmilli, strlen(tokenmilli)) == 0){
			continue;
		}
		else if(_strnicmp(i_line,tokentno, strlen(tokentno)) == 0){
			nowtno = atoi(strtok(&i_line[4],"\t\n"));
			continue;
		}
		else {
		
			traj.milli = atoi(strtok(&i_line[0],","));
			traj.fno = atoi(strtok(NULL, ","));
			
			traj.gp.x = atof(strtok(NULL, ","));
			traj.gp.y = atof(strtok(NULL, ","));
			traj.len0 = atof(strtok(NULL, ","));
			traj.len1 = atof(strtok(NULL, ","));
			traj.gv1.x = atof(strtok(NULL, ","));
			traj.gv1.y = atof(strtok(NULL, ","));
			traj.gv2.x = atof(strtok(NULL, ","));
			traj.gv2.y = atof(strtok(NULL, ","));
			tmpfno = FindDsFno(traj.milli);
			if(tmpfno == -1)
				continue;
			
			//tmpfno = traj.fno;
			
			//fprintf(fp2,"%d %d %f %f  %f %f\n",traj.fno,nowtno,traj.gp.x,traj.gp.y,traj.len0,traj.len1);


			GetRectPts(&traj.gp,&traj.gv1,traj.len0,traj.len1,c,0);
			shv.x = -dsDatBuf.scandata[tmpfno].shv.x;
			shv.y = -dsDatBuf.scandata[tmpfno].shv.y;
			rot[0][0] = cos(dsDatBuf.scandata[tmpfno].ang.z);
			rot[0][1] = sin(dsDatBuf.scandata[tmpfno].ang.z);
			rot[1][0] = -sin(dsDatBuf.scandata[tmpfno].ang.z);
			rot[1][1] = cos(dsDatBuf.scandata[tmpfno].ang.z);
			for(i=0;i<4;i++){
				pt[i].x = c[i].x;
				pt[i].y = c[i].y;
				shiftPoint2d(&pt[i],&shv);
				rotatePoint2d(&pt[i],rot);
			/*	if(i==0){
					lpt[0].x = pt[i].x;
					lpt[0].y = pt[i].y;
					lpt[1].x = pt[i].x;
					lpt[1].y = pt[i].y;
				}
				else {
					if(pt[i].x < lpt[0].x){
						lpt[0].x = pt[i].x;
						lpt[0].y = pt[i].y;
					}
					if(pt[i].x > lpt[1].x){
						lpt[1].x = pt[i].x;
						lpt[1].y = pt[i].y;
					}
				}
				*/
			}
			//lpt[0].x = traj.gp.x;
			//lpt[0].y = traj.gp.y;
			//shiftPoint2d(&lpt[0],&shv);
			//rotatePoint2d(&lpt[0],rot);

			for(i=0;i<4;i++){
				pL.x = pt[i].x;
				pL.y = pt[i].y;
				pL.z = 1.1;
				dpL.x = pt[i].x;
				dpL.y = pt[i].y;
				dpL.z = -0.7;
				rotatePoint3d(pL,lmsTrans.rot);
				shiftPoint3d(pL,lmsTrans.shv);
				rotatePoint3d(dpL,lmsTrans.rot);
				shiftPoint3d(dpL,lmsTrans.shv);
				// 转换到相机坐标，先平移再旋转
				shiftPoint3d(pL,cameraTrans.invshv);
				rotatePoint3d(pL,cameraTrans.invrot);
				shiftPoint3d(dpL,cameraTrans.invshv);
				rotatePoint3d(dpL,cameraTrans.invrot);


				normalPoint3d(pL);
				normalPoint3d(dpL);
				if (pL.y>=0)
					longitude = acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
				else
					longitude = -acos(pL.x/sqrt(pL.x*pL.x+pL.y*pL.y));
				latitude = asin(pL.z);

				if (dpL.y>=0)
					long2 = acos(dpL.x/sqrt(dpL.x*dpL.x+dpL.y*dpL.y));
				else
					long2 = -acos(dpL.x/sqrt(dpL.x*dpL.x+dpL.y*dpL.y));
				lati2 = asin(dpL.z);

				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].apt[i].x = longitude;
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].apt[i].y = latitude;
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].adpt[i].x = long2;
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].adpt[i].y = lati2;
				
				
				longitude = (1 - longitude/M_PI) /2 ;
				latitude = 0.5 - latitude/M_PI;
				x = int(longitude*2048);
				y = int(latitude*1024);

				long2 = (1 - long2/M_PI) /2 ;
				lati2 = 0.5 - lati2/M_PI;
				x2 = int(long2*2048);
				y2 = int(lati2*1024);
			
				if (x<0)	x = 0;
				if (x>=2048)	x = 2048-1;
				if (y<0)	y = 0;
				if (y>=1024)	y = 1024-1;
				if (x2<0)	x2 = 0;
				if (x2>=2048)	x2 = 2048-1;
				if (y2<0)	y2 = 0;
				if (y2>=1024)	y2 = 1024-1;
				
				//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].pt[i].x = x;
				//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].pt[i].y = y;
				//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].dpt[i].x = x2;
				//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].dpt[i].y = y2;
				
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].pt[i].x = x;
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].pt[i].y = y;
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].dpt[i].x = x2;
				trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].dpt[i].y = y2;

			}
			pp.x = traj.gp.x;
			pp.y = traj.gp.y;

			shiftPoint2d(&pp,&shv);
			rotatePoint2d(&pp,rot);
			p.x = pp.x;
			p.y = pp.y;
			p.z = 0;
			rotatePoint3d(p,lmsTrans.rot);
			shiftPoint3d(p,lmsTrans.shv);
			shiftPoint3d(p,cameraTrans.invshv);
			rotatePoint3d(p,cameraTrans.invrot);
			normalPoint3d(p);
			if (p.y>=0)
				longitude = acos(p.x/sqrt(p.x*p.x+p.y*p.y));
			else
				longitude = -acos(p.x/sqrt(p.x*p.x+p.y*p.y));
			latitude = asin(p.z);
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].ang = longitude;
			longitude = (1 - longitude/M_PI) /2 ;
			latitude = 0.5 - latitude/M_PI;
			x = int(longitude*2048);
			y = int(latitude*1024);
			if (x<0)	x = 0;
			if (x>=2048)	x = 2048-1;
			if (y<0)	y = 0;
			if (y>=1024)	y = 1024-1;
			//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].p.x = x;
			//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].p.y = y;
			//dsDatBuf.scandata[tmpfno].traj[dsDatBuf.scandata[tmpfno].trajnum].tno = nowtno;
			//dsDatBuf.scandata[tmpfno].trajnum ++;	
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].gp.x = p.x;
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].gp.y = p.y;
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].v.x = traj.gv1.x;
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].v.y = traj.gv1.y;
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].p.x = x;
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].p.y = y;
			trajs.trajs[traj.fno].traj[trajs.trajs[traj.fno].trajnum].tno = nowtno;
			trajs.trajs[traj.fno].milli = traj.milli;
			trajs.trajs[traj.fno].trajnum ++;
			if(trajs.num < traj.fno)
				trajs.num = traj.fno;
		}	
	} while (fgets (i_line, 200, fp));
	//fclose(fp2);
	for(i=2;i<trajs.num+1;i++){
		if(trajs.trajs[i].milli == 0){
			trajs.trajs[i].milli = trajs.trajs[i-1].milli;
			trajs.trajs[i].trajnum = trajs.trajs[i-1].trajnum;
			for(x=0;x<trajs.trajs[i].trajnum;x++){
				trajs.trajs[i].traj[x].dpt[0]=trajs.trajs[i-1].traj[x].dpt[0];
				trajs.trajs[i].traj[x].dpt[1]=trajs.trajs[i-1].traj[x].dpt[1];
				trajs.trajs[i].traj[x].dpt[2]=trajs.trajs[i-1].traj[x].dpt[2];
				trajs.trajs[i].traj[x].dpt[3]=trajs.trajs[i-1].traj[x].dpt[3];
				trajs.trajs[i].traj[x].p=trajs.trajs[i-1].traj[x].p;
				trajs.trajs[i].traj[x].pt[0]=trajs.trajs[i-1].traj[x].pt[0];
				trajs.trajs[i].traj[x].pt[1]=trajs.trajs[i-1].traj[x].pt[1];
				trajs.trajs[i].traj[x].pt[2]=trajs.trajs[i-1].traj[x].pt[2];
				trajs.trajs[i].traj[x].pt[3]=trajs.trajs[i-1].traj[x].pt[3];
				trajs.trajs[i].traj[x].tno=trajs.trajs[i-1].traj[x].tno;
				trajs.trajs[i].traj[x].v=trajs.trajs[i-1].traj[x].v;
				trajs.trajs[i].traj[x].gp=trajs.trajs[i-1].traj[x].gp;
				trajs.trajs[i].traj[x].apt[0]=trajs.trajs[i-1].traj[x].apt[0];
				trajs.trajs[i].traj[x].apt[1]=trajs.trajs[i-1].traj[x].apt[1];
				trajs.trajs[i].traj[x].apt[2]=trajs.trajs[i-1].traj[x].apt[2];
				trajs.trajs[i].traj[x].apt[3]=trajs.trajs[i-1].traj[x].apt[3];
				trajs.trajs[i].traj[x].adpt[0]=trajs.trajs[i-1].traj[x].adpt[0];
				trajs.trajs[i].traj[x].adpt[1]=trajs.trajs[i-1].traj[x].adpt[1];
				trajs.trajs[i].traj[x].adpt[2]=trajs.trajs[i-1].traj[x].adpt[2];
				trajs.trajs[i].traj[x].adpt[3]=trajs.trajs[i-1].traj[x].adpt[3];
				
			}
		}
	}
	fclose(fp);
	return true;
}

int LMS::getTrajNum(int fno)
{
	return trajs.trajs[fno].trajnum;
	//return dsDatBuf.scandata[fno].trajnum;
}

pointdisp LMS::getTrajPt(int fno, int t,int i)
{
	return trajs.trajs[fno].traj[t].pt[i];
//	return dsDatBuf.scandata[fno].traj[t].pt[i];
}

pointdisp LMS::getTrajDPt(int fno, int t,int i)
{
	return trajs.trajs[fno].traj[t].dpt[i];
	//return dsDatBuf.scandata[fno].traj[t].dpt[i];
}

point2d LMS::getGTrajPt(int fno, int t,int i)
{
	return trajs.trajs[fno].traj[t].apt[i];
	//	return dsDatBuf.scandata[fno].traj[t].pt[i];
}

point2d LMS::getGTrajDPt(int fno, int t,int i)
{
	return trajs.trajs[fno].traj[t].adpt[i];
	//return dsDatBuf.scandata[fno].traj[t].dpt[i];
}

pointdisp LMS::getTrajP(int fno, int t)
{
	return trajs.trajs[fno].traj[t].p;
	//return dsDatBuf.scandata[fno].traj[t].p;
}

point2d LMS::getTrajV(int fno,int t)
{
	return trajs.trajs[fno].traj[t].v;
}

point2d LMS::getTrajGP(int fno,int t)
{
	return trajs.trajs[fno].traj[t].gp;
}

double LMS::getTrajAng(int fno,int t)
{
	return trajs.trajs[fno].traj[t].ang;
}

int LMS::getTrajTno(int fno,int t)
{
	return trajs.trajs[fno].traj[t].tno;
}

int LMS::FindTrajTimeNo(long millisec)
{
	long starttime = trajs.trajs[1].milli;
	long endtime = trajs.trajs[trajs.num].milli;
	if(millisec < starttime){
		//qDebug("Pgr is earlier than Lms!!");
		return -1;
	}
	if(millisec > endtime){
		//qDebug("Pgr is later than Lms!!");
		return -1;
	}
	int tmpfno=1;
	//tmpfno = (millisec - starttime)/(lmsDatBuf.scandata[1].Millisecond-starttime);
	for(int i=1; i<trajs.num;i++){
		if(millisec >= trajs.trajs[i].milli &&millisec < trajs.trajs[i+1].milli)
			return i;
	}
	return -1;
	/*
	while(1){
		if(millisec >= trajs.trajs[tmpfno].milli &&millisec < trajs.trajs[tmpfno+1].milli)
			break;
		else if(millisec < trajs.trajs[tmpfno].milli)
			tmpfno --;
		else if(millisec >= trajs.trajs[tmpfno+1].milli)
			tmpfno ++;
	}
	return tmpfno;*/

}
/*
point2d LMS::getLPt(int fno,int dno)
{
	return dsDatBuf.scandata[fno].lpt[dno];
}
point2d LMS::getLPt1(int fno,int dno)
{
	return dsDatBuf1.scandata[fno].lpt[dno];
}
point2d LMS::getLPt2(int fno,int dno)
{
	return dsDatBuf2.scandata[fno].lpt[dno];
}
point2d LMS::getLPt3(int fno,int dno)
{
	return dsDatBuf3.scandata[fno].lpt[dno];
}*/