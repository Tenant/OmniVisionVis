#ifndef LMS_H
#define LMS_H

#include "../Transform_fang/transform_fang.h"
#include "../define_zhao.h"
#include <windows.h>

#define lmsNumber 1

class LMS{

private:
	LMSHEADER lmsHeader[4];
	LMSDATBUF lmsDatBuf[4];
	TRANSINFO lmsTrans;
	TRANSINFO cameraTrans;
	DSHEADER  dsHeader;
	DSHEADER  dsHeader1;
	DSHEADER  dsHeader2;
	DSHEADER  dsHeader3;
	DSDATBUF  dsDatBuf;
	DSDATBUF  dsDatBuf1;
	DSDATBUF  dsDatBuf2;
	DSDATBUF  dsDatBuf3;

public:
	TRAJS    trajs;

public:
	LMS();
	~LMS();

	void initTrans();
	void CalcTrans(TRANSINFO& trans);
	
	bool LoadLmsDataFromFile(const char *filename);
	bool LoadDsDataFromFile(char *filename);
	bool LoadTrajFile(char *filename);

	long getCurrentTime(int ldno, int index);
	long getStartTime(int ldno);
	long getEndTime(int ldno);
	int getDatNum(int ldno);
	long getDsStartTime();
	long getDsEndTime();
	int getDsDatNum();
	int getDsDatNum1();
	int getDsDatNum2();
	int getDsDatNum3();
	double getDsV(int fno);
	point2d getDsP(int fno);
	void LmsptOnPgr(int ldno,int fno, int row,int col);
	void DsptOnPgr(int fno, int row,int col);
	void DsptOnPgr1(int fno, int row,int col);
	void DsptOnPgr2(int fno, int row,int col);
	void DsptOnPgr3(int fno, int row,int col);
	
	bool FindFno(int ldno,long millisec, int& index);
	int FindDsFno(long millisec);
	double getDistance(int ldno,int fno, int dno);
	point2d getLocalpt(int ldno, int fno, int dno);
	pointdisp getPgrpt(int ldno, int fno, int dno);
	double getDsDistance(int fno, int dno);
	double getDsDistance1(int fno, int dno);
	double getDsDistance2(int fno, int dno);
	double getDsDistance3(int fno, int dno);
	pointdisp getDsPgrpt(int fno, int dno);
	pointdisp getDsPgrpt1(int fno, int dno);
	pointdisp getDsPgrpt2(int fno, int dno);
	pointdisp getDsPgrpt3(int fno, int dno);
	/*point2d getLPt(int fno,int dno);
	point2d getLPt1(int fno,int dno);
	point2d getLPt2(int fno,int dno);
	point2d getLPt3(int fno,int dno);*/
	int	getLabel(int ldno,int fno, int dno);
	int getTrajNum(int fno);
	pointdisp getTrajPt(int fno, int t,int i);
	pointdisp getTrajDPt(int fno, int t,int i);
	point2d getGTrajPt(int fno, int t,int i);
	point2d getGTrajDPt(int fno, int t,int i);
	pointdisp getTrajP(int fno, int t);
	double    getTrajAng(int fno,int t);
	int			FindTrajTimeNo(long millisec);
	point2d getTrajV(int fno,int t);
	point2d getTrajGP(int fno,int t);
	int     getTrajTno(int fno,int t);
};


#endif