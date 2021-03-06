#include "GTLabel.h"

bool GTLabel::init()
{
	cv::FileStorage fs("config.yml", cv::FileStorage::READ);  
	if (!fs.isOpened())
		return false;
	fs["gtFile"] >> outputFile;
	fs["responseTime"] >> _responseTime;
	fs.release();


#ifdef DEBUG_VIS
	extraOutputFile.open(outputFile + std::string(".extra.csv"), std::ios::app);
	if (!extraOutputFile.is_open())
		return false;
	extraOutputFile << "ResizeLadybugRadio: " << ResizeLadybugRadio << std::endl;
#endif

	auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	struct tm* ptm = localtime(&tt);
	char timestr[60] = { 0 };
	sprintf(timestr, "%d-%02d-%02d-%02d-%02d-%02d",
		(int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
		(int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
#ifdef DEBUG_VIS
	extraOutputFile << "Time: " << timestr << std::endl;
#endif

	if (!recorder.load(outputFile))
	{
		return false;
	}
	tracker.uid = recorder.getSavedMaxUID();
	curMaxUID = tracker.uid;

	std::cout << "Select ROI with (mouse), refine it with [a][s][d][w] and (scroll), then start with [Enter]/[Space].\n";
	std::cout << "Press [r] to re-initialize and [ESC] to exit.\n";

	tracker_flag = TS_OFF;

	_refineStartTime = 0;
	_refineEndTime = 0;

	return true;
}

void GTLabel::refineSavedGT_mono(Flea2Reader& flea2, GPSReader& gps, Flea2Data& monoData, VelodyneData& veloData, cv::Mat& canvas_mono, cv::Mat& canvas_bv, bool& isAddNew, int uid)
{
	isAddNew = false;

	cv::Point3d objPos;
	int textHeight = -7;

	if (!tracker.isInit)
	{
		labeler.label(tracker.roi, monoData.img, winNameMono);
		tracker.init(tracker.roi, monoData.img);
		tracker.isInit = true;

		cv::rectangle(canvas_mono, tracker.roi, CV_RGB(0, 0, 255));
		std::stringstream ssTmp;
		ssTmp.str("");
		ssTmp << uid << ": " << _curInfo.name;
		cv::putText(canvas_mono, ssTmp.str(), cv::Point2i(tracker.roi.x, tracker.roi.y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(0, 0, 255), 2);
	}
	else
	{
		float score;
		tracker.roi = tracker.update(monoData.img, score);

		cv::rectangle(canvas_mono, tracker.roi, CV_RGB(0, 0, 255));
		std::stringstream ssTmp;
		ssTmp.str("");
		ssTmp << uid << ": " << _curInfo.name;
		cv::putText(canvas_mono, ssTmp.str(), cv::Point2i(tracker.roi.x, tracker.roi.y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(0, 0, 255), 2);
	}

	cv::imshow(winNameMono, canvas_mono);
	char key = cv::waitKey(0);
	int saved_gt_index;
	OneGroundTruth onegt;
	bool isDeleted;
	cv::Mat monoData_img = monoData.img.clone();
	cv::Mat _HSV;
	vector<Mat> hsvSplit;
	switch (key)
	{
	case 'r':
	case 'R':
		//转换为HSV格式，对V分量进行均衡化
		cvtColor(monoData_img, _HSV, cv::COLOR_BGR2HSV);
		split(_HSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, _HSV);
		cvtColor(_HSV, monoData_img, cv::COLOR_HSV2BGR);
		labeler.label(tracker.roi, monoData_img, winNameMono);
		tracker.init(tracker.roi, monoData.img);
		//break; r就自然要修改
	case 'g':
	case 'G':
		if (recorder.getOneSavedGTIndex('C', uid, monoData.timestamp, saved_gt_index))
		{
			onegt = recorder.getOneSavedGT(saved_gt_index);
			
			//if (isValidROI(enlargedRegion, img.size())) //似乎没有可能没有重叠
			cv::Rect trackedROI = cutValidROI(tracker.roi, monoData.img.size());
			onegt.objCorners[0] = cv::Point2d(trackedROI.x, trackedROI.y);
			onegt.objCorners[1] = cv::Point2d(trackedROI.x + trackedROI.width, trackedROI.y);
			onegt.objCorners[2] = cv::Point2d(trackedROI.x + trackedROI.width, trackedROI.y + trackedROI.height);
			onegt.objCorners[3] = cv::Point2d(trackedROI.x, trackedROI.y + trackedROI.height);
			recorder.setOneSavedGT(onegt, saved_gt_index);

			isAddNew = false;
		}
		else
		{
			tracker.uid = uid;
			isAddNew = true;
		}
		break;
	case 27:
		recorder.save(outputFile);
		break;
	case 'd':
	case 'D':
		isDeleted = recorder.erase('C', uid, monoData.timestamp);
		//recorder.save(outputFile);
		break;
	default:
		break;
	}
}

void GTLabel::visualiseLastArchor(cv::Mat& canvas, GPSReader& gps)
{
	if (!_mapArchorGPSPositions.empty())
	{
		std::cout << "Archor num: " << _mapArchorGPSPositions.size() << std::endl;
		cv::Point3d vehicleP;
		cv::Point2d globalPos = cv::Point2d((*_mapArchorGPSPositions.rbegin()).second.x, (*_mapArchorGPSPositions.rbegin()).second.y);
		gps.GlobalP2VehicleP(globalPos, vehicleP);

		cv::Point2d bvp;
		bvp.x = vehicleP.x / _pixelSize + _mapSize / 2;
		bvp.y = _mapSize / 2 - vehicleP.y / _pixelSize;

		cv::circle(canvas, bvp, 5, CV_RGB(255, 255, 0));
		cv::imshow(winNameBV, canvas);
		//cv::line(canvas, bvp[3], bvp[0], CV_RGB(255, 0, 0));
		/*std::stringstream ssTmp;
		ssTmp.str("");
		ssTmp << gt.uid << ": " << gt.objClass;
		cv::putText(canvas_bv, ssTmp.str(), cv::Point2i(bvp[0].x, bvp[0].y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);*/
	}
}

bool GTLabel::getRefineSavedGT_timeDuration(int uid, long long curtime)
{
	if (_refineStartTime != 0 && _refineEndTime != 0)//假如已经找到了就不找了
		return true;
	for (auto gt : recorder.gts)
	{
		if (gt.uid == uid)
		{
			_refineStartTime = gt.visibleMinStartTime;
			_refineEndTime = gt.visibleMaxEndTime;
			if (curtime > _refineStartTime)
			{
				//假如开始找的时候，第一帧的时间戳比_refineStartTime还后说明有东西没有refine
				std::cout << "startTime should be earlier than " << _refineStartTime << std::endl;
				system("pause");
				return false;
			}
			_mapArchorGPSPositions.clear();
			return true;
		}
	}
	return false;
}

void GTLabel::refineSavedGT_bv_addArchor(GPSReader& gps, VelodyneData& veloData, int uid)
{
	int index;
	if (recorder.getOneSavedGTIndex('L', uid, veloData.timestamp, index))
	{
		GPSData fakeGPSPos;
		fakeGPSPos.x = recorder.gts[index].objPos.x;
		fakeGPSPos.y = recorder.gts[index].objPos.y;
		fakeGPSPos.yaw = recorder.gts[index].generateYawFromCorners();
		fakeGPSPos.yaw = setGlobalYaw(fakeGPSPos.yaw, gps.getCurrentData());
		fakeGPSPos.timestamp = veloData.timestamp;

		_mapArchorGPSPositions.insert(std::make_pair(veloData.timestamp, fakeGPSPos));
	}
}

void GTLabel::refineSavedGT_bv_addFakeArchor(GPSReader & gps, VelodyneData & veloData, int uid, cv::Mat& canvas_bv, Flea2Reader& flea2)
{
	int index;
	if (recorder.getOneSavedGTIndex('L', uid, veloData.timestamp, index))
	{
		OneGroundTruth gt = recorder.gts[index];

		double yawOfThisGT = gt.generateYawFromCorners();
		setGlobalYaw(yawOfThisGT, gps.getCurrentData());//实时更新globalYaw, 这边相当于只是算出来全局的yaw 然后去插值用
		gt.objYaw = getLocalYaw(gps.getCurrentData());

		GPSData fakeGPSPos;

		cv::Rect roi;
		fang::selectROI(roi, fakeGPSPos.yaw, gt, canvas_bv, flea2, gps);
		fakeGPSPos.yaw = setGlobalYaw(fakeGPSPos.yaw, gps.getCurrentData());
		fakeGPSPos.x = gt.objPos.x;
		fakeGPSPos.y = gt.objPos.y;
		fakeGPSPos.timestamp = veloData.timestamp;

		_mapArchorGPSPositions.insert(std::make_pair(veloData.timestamp, fakeGPSPos));

		//朝向插值太麻烦...我不想做...必须要做
	}
}

void GTLabel::modifyOneSavedGT(OneGroundTruth& gt, GPSData refinedCenter, GPSReader& gps)
{
	if (!gps.grabData(gt.visibleStartTime))
		system("pause");
	if (isSensorTypeMatched(gt, 'C'))
	{
		gt.objPos.x = refinedCenter.x;
		gt.objPos.y = refinedCenter.y;
	}
	else
	{
		gt.objPos.x = refinedCenter.x;
		gt.objPos.y = refinedCenter.y;
		//计算四个角点	
		setGlobalYaw(refinedCenter.yaw + gps.getCurrentData().yaw, gps.getCurrentData());//这里才是把插好值的全局的yaw转换到局部的yaw
		double localy = getLocalYaw(gps.getCurrentData());
		//std::cout << gt.objYaw*180.0 / CV_PI << ' ' << localy*180.0 / CV_PI << std::endl;
		GTClassInfo gtci = GTClassInfo(gt.objClass);
		cv::Point3d fake3dP;
		gps.GlobalP2VehicleP(gt.objPos, fake3dP);
		//fake3dP.x = gt.objPos.x;
		//fake3dP.y = gt.objPos.y;
		fake3dP.z = gtci.height/2;
		Flea2Reader test = Flea2Reader();
		generate2DBBox(gtci, fake3dP, localy, gt.sensorType, test, gt.objCorners);

	}
}

void GTLabel::refineSavedGT_bv_startSailing(GPSReader& gps, int uid)
{
	if (_mapArchorGPSPositions.empty())
		return;
	for (auto& gt : recorder.gts)
	{
		if (gt.uid == uid)
		{
			GPSData refinedCenter;
			if (_mapArchorGPSPositions.size() == 1)
			{
				//refinedCenter = (*_mapArchorGPSPositions.begin()).second;
				refinedCenter = (*_mapArchorGPSPositions.begin()).second;
			}
			else
			{
				if (gt.visibleStartTime < (*_mapArchorGPSPositions.begin()).first)//早于第一个锚点
				{
					refinedCenter = (*_mapArchorGPSPositions.begin()).second;
				}
				else if (gt.visibleStartTime >= (*_mapArchorGPSPositions.rbegin()).first)//晚于最后第一个锚点
				{
					refinedCenter = (*_mapArchorGPSPositions.rbegin()).second;
				}
				else //在范围内需要插值
				{
					auto nextE = _mapArchorGPSPositions.upper_bound(gt.visibleStartTime);
					auto prevE = std::prev(nextE);
					refinedCenter = getInterpolatedGPS_YawXY(gt.visibleStartTime, (*prevE).second, (*nextE).second);
				}
			}
			modifyOneSavedGT(gt, refinedCenter, gps);
		}
	}
	recorder.save(outputFile);
}

void GTLabel::addMissingGT_bv(GPSReader& gps, Flea2Reader& flea2, VelodyneData& veloData, int uid, GTClassInfo gtci)
{
	//todo
	int index;
	long long time = veloData.timestamp;
	if (time >= _refineStartTime && time <= _refineEndTime)
	{
		if (!recorder.getOneSavedGTIndex('L', uid, time, index))
		{
			accurateBBox abbox;
			abbox.nearMost = cv::Point3d(0, 0, 0);//空的
			OneGroundTruth gt;
			gt.gtID = uid;
			gt.visibleStartTime = veloData.timestamp;
			gt.visibleEndTime = veloData.timestamp + oneVeloFrameVisibleTimeDuration;
			gt.sensorType = 'L';
			gt.objClass = gtci.objClass;
			gt.objYaw = getLocalYaw(gps.getCurrentData());//其实可能有bug: 假如第一次进来就在补漏，那globalYaw是随机的
			generate2DBBox(gtci, abbox.nearMost, gt.objYaw, gt.sensorType, flea2, gt.objCorners);
			gt.uid = uid;
			cv::Point3d objPos = cv::Point3d(0, 0, gtci.height/2);
			gps.VehicleP2GlobalP(objPos, gt.objPos);

			recorder.update(gt);//因为已经判断getOneSavedGTIndex了，所以添加的肯定是没有的
			//addNewGT(gt);
		}
		else
		{
			double yawOfThisGT = recorder.gts[index].generateYawFromCorners();
			
			setGlobalYaw(yawOfThisGT, gps.getCurrentData());//实时更新globalYaw
		}
	}
	else if (time > _refineEndTime)
	{
		printf("esc to exit and save\n");
		char key = cv::waitKey(0);
		if (key == 27)
			recorder.save(outputFile);
		else
		{
			printf("did not save!!!\n");
			system("pause");
			exit(0);
		}
	}
}

void GTLabel::visualizeSavedGT(Flea2Reader& flea2, GPSReader& gps, Flea2Data & monoData, VelodyneData & veloData, cv::Mat & canvas_mono, cv::Mat & canvas_bv)
{
#define reGenerateCorners 0
	std::vector<OneGroundTruth> saved_gt;
	cv::Point3d objPos;
	int textHeight = -7;
	if (recorder.getSavedGT('C', monoData.timestamp, saved_gt))
	{
		//可视化在mono， 重复实现了， 单独提取出来变成函数
		for (auto gt : saved_gt)
		{
#if reGenerateCorners
			gps.GlobalP2VehicleP(gt.objPos, objPos);
			curInfo = GTClassInfo(gt.objClass);
			generate2DBBox(objPos, gt.sensorType, flea2, gt.objCorners);
#endif
			gt.draw(canvas_mono, CV_RGB(255, 0, 0));
			std::stringstream ssTmp;
			ssTmp.str("");
			ssTmp << gt.uid << ": " << gt.objClass;
			cv::putText(canvas_mono, ssTmp.str(), cv::Point2i(gt.objCorners[0].x, gt.objCorners[0].y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);
		}
		cv::imshow(winNameMono, canvas_mono);
	}
	if (recorder.getSavedGT('L', veloData.timestamp, saved_gt))
	{
		//std::cout << "saved gt l number: " << saved_gt.size() << std::endl;
		//可视化在bv， 重复实现了， 单独提取出来变成函数
		for (auto gt : saved_gt)
		{
#if reGenerateCorners
			gps.GlobalP2VehicleP(gt.objPos, objPos);
			curInfo = GTClassInfo(gt.objClass);
			generate2DBBox(objPos, gt.sensorType, flea2, gt.objCorners);
#endif
			gt.draw_bv(canvas_bv, CV_RGB(255, 0, 0));
			std::stringstream ssTmp;
			ssTmp.str("");
			ssTmp << gt.uid << ": " << gt.objClass;
			cv::Point2i bvp;
			bvp.x = gt.objCorners[0].x / _pixelSize + _mapSize / 2;
			bvp.y = _mapSize / 2 - gt.objCorners[0].y / _pixelSize;
			cv::putText(canvas_bv, ssTmp.str(), cv::Point2i(bvp.x, bvp.y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);
		}
		cv::imshow(winNameBV, canvas_bv);
	}
}

void GTLabel::verify(const cv::Mat& m)
{
	//添加说明...
	bool isValid = false;
	bool isVerified = false;
	while (!isVerified)
	{
		char key = cv::waitKey(10);
		switch (key)
		{
		case 'H':
		case 'h':
			_curInfo = GT_HUMAN;
			isValid = true;
			break;
		case 'V':
		case 'v':
			_curInfo = GT_VEHICLE;
			isValid = true;
			break;
		case 'B':
		case 'b':
			_curInfo = GT_BOX;
			isValid = true;
			break;
		case 'O':
		case 'o':
			_curInfo = GT_OBS;
			isValid = true;
			break;
		case 13://回车
		case 20://空格
		case 32://
			if(isValid == true)
				isVerified = true;
			break;
		case -1:
			break;//什么都没按
		default:
			isValid = false;//不能输入其他按键
			break;
		}
		//分不同情况可视化
		cv::Mat canvas;
		m.copyTo(canvas);
		printOnMat(canvas);
		cv::imshow(verificationWinName, canvas);
	}
}

void GTLabel::printOnMat(cv::Mat & canvas)
{
	std::stringstream ssTmp;
	ssTmp.str("");
	ssTmp// << uid << ": " 
		<< _curInfo.name;
	cv::putText(canvas, ssTmp.str(), cv::Point(4, 13), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);
}

double GTLabel::getLocalYaw(const GPSData& gps)
{
	double localYaw = globalYaw + gps.yaw;
	if (localYaw < 0)
		localYaw += CV_2PI;
	else if(localYaw > CV_2PI)
		localYaw -= CV_2PI;
	return localYaw;
}

double GTLabel::setGlobalYaw(double localYaw, const GPSData& gps)
{
	//IMU 是正北为0， 往东+， 往西-
	globalYaw = localYaw - gps.yaw;
	if (globalYaw < 0)
		globalYaw += CV_2PI;
	else if (globalYaw > CV_2PI)
		globalYaw -= CV_2PI;
	return globalYaw;
}

void GTLabel::generateImageForVerification(const cv::Mat& m, cv::Mat& out)
{
	cv::Rect roi = cutValidROI(tracker.roi, m.size());
	if (roi.width < roi.height)
	{
		out.create(cv::Size(verificationMinSize, verificationMinSize*(double)roi.height / (double)roi.width), m.type());
		cv::resize(m(roi), out, out.size());
	}
	else
	{
		out.create(cv::Size(verificationMinSize*(double)roi.width / (double)roi.height, verificationMinSize), m.type());
		cv::resize(m(roi), out, out.size());
	}
}

void GTLabel::interaction(accurateBBox abbox, double localYaw, Flea2Reader& flea2, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv, TrackingStatus& lastStatus)
{
	//log
	/*std::stringstream convertor;
	convertor.str("");
	convertor << curPos << " / " << _totalFrameNum << "; "
	<< std::setprecision(4) << 100 * static_cast<double>(curPos) / static_cast<double>(_totalFrameNum) << "%";
	putText(canvas, convertor.str(), Point(4, 20), FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);*/

	//double rot_angle = curInfo.yaw;
	double oWidth = _curInfo.width;
	double oHeight = _curInfo.height;
	double oLength = _curInfo.length;

	//可视化在pano
	putText(canvas_pano, "Press A to repaint", cv::Point(4, 20 * 3), cv::FONT_HERSHEY_SIMPLEX, 2, CV_RGB(255, 0, 0), 2);
	rectangle(canvas_pano, tracker.roi, CV_RGB(0, 0, 255));
	imshow(winNamePano, canvas_pano);

	if (isTrackerInit())
	{
		//可视化在mono， 重复实现了， 单独提取出来变成函数
		OneGroundTruth onegt;
		onegt.sensorType = 'C';
		generate2DBBox(_curInfo, abbox.nearMost, localYaw, onegt.sensorType, flea2, onegt.objCorners);
		onegt.draw(canvas_mono, CV_RGB(255, 0, 0));

		//可视化在bv， 重复实现了， 单独提取出来变成函数
		onegt.sensorType = 'L';
		generate2DBBox(_curInfo, abbox.nearMost, localYaw, onegt.sensorType, flea2, onegt.objCorners);
		onegt.draw_bv(canvas_bv, CV_RGB(255, 0, 0));
}
	cv::imshow(winNameMono, canvas_mono);
	cv::imshow(winNameBV, canvas_bv);



	char key = cv::waitKey(_responseTime);
	switch (key)
	{
	case 'n':
	case 'N':
		lastStatus = TS_INIT;
		break;
	case 'r':
	case 'R':
		lastStatus = TS_REINIT;
		break;
	case 27:
		lastStatus = TS_EXIT;
		recorder.save(outputFile);
		break;
	case 'g':
	case 'G':
		lastStatus = TS_OFF;
		break;
	default:
		;
		/*if (lastStatus == TS_OFF)
		return TS_OFF;
		return TS_REFINE;*/
	}
}

void GTLabel::labelSingleFrame(LadybugReader& ladybug, Flea2Reader& flea2, GPSData& gpsData, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv)
{
	int shift;
	cv::Mat ori_pano;
	ladybug.getCurrentData().img.copyTo(ori_pano);
	

	if (tracker_flag == TS_TRACKING)
	{
		tracker.roi.x -= shiftedImageForTracking.getShift();//先加
		shift = shiftedImageForTracking.getShift() + tracker.roi.x + tracker.roi.width - ori_pano.cols / 2;
		shiftedImageForTracking.setShift(shift, ori_pano);
		tracker.roi = tracker.update(shiftedImageForTracking.image, tracker.score);


		generateImageForVerification(shiftedImageForTracking.image, _verification);
		cv::imshow(verificationWinName, _verification);

		tracker.roi.x += shiftedImageForTracking.getShift();
		if (tracker.roi.x > ori_pano.cols)
			tracker.roi.x -= ori_pano.cols;
	}

	//放外面是因为懒得传参数
	//getRoughPosPano(getTRect(), veloData, ladybug, objPos);
	accurateBBox abbox;
	getAccuratePosPano(getTRect(), veloData, ladybug, abbox);
	double localYaw = getLocalYaw(gpsData);
	//std::cout << "GPS: " << gpsData.yaw / CV_PI * 180.0 << ", ";
	//std::cout << "Global: " << globalYaw / CV_PI * 180.0 << ", ";
	//std::cout << "Local: " << localYaw / CV_PI * 180.0 << std::endl;
	interaction(abbox, localYaw, flea2,canvas_pano, canvas_mono, canvas_bv, tracker_flag);//跟踪开始后唯一的键盘响应

	switch (tracker_flag)
	{
	case TS_INIT:
		//tracker.uid++;
		//curMaxUID = tracker.uid;
		//和re-init的唯一区别就是这个uid++
	case TS_REINIT:
		double gYaw;
		labeler.labelMore(tracker.roi, gYaw, ladybug, flea2, veloData, canvas_pano, canvas_mono, canvas_bv);
		setGlobalYaw(gYaw, gpsData);

		generateImageForVerification(ori_pano, _verification);
		verify(_verification);// 确认类型

		shift = tracker.roi.x + tracker.roi.width - ori_pano.cols/2;
		shiftedImageForTracking.setShift(shift, ori_pano);

		tracker.roi.x -= shiftedImageForTracking.getShift();
		tracker.init(tracker.roi, shiftedImageForTracking.image);//ori_pano);
		tracker.roi.x += shiftedImageForTracking.getShift();
		tracker.isInit = true;

		tracker_flag = TS_TRACKING;

		break;
		//re init也要记录啊,所以不break
		//re init还没算出来三维位置，暂时不记录

	case TS_TRACKING:
		//recorder.update(bug, tracker.roi, ori_pano.size());
		break;
	case TS_OFF:
		tracker.isInit = false;
		//recorder.update(bug, recorder.gtBBox(bug), ori_pano.size());
		break;
	case TS_EXIT:
		tracker.isInit = false;
		break;
	}

	if (tracker_flag == TS_TRACKING)
	{
#ifdef DEBUG_VIS
		extraOutputFile << ladybug.getCurrentData().timestamp << ','
			<< tracker.uid << ','
			<< tracker.roi.x << ',' << tracker.roi.y << ','
			<< tracker.roi.width << ',' << tracker.roi.height << std::endl;
#endif
	}
}

void GTLabel::addNewGT(OneGroundTruth onegt)
{
	for (auto gt : recorder.gts)
	{
		if ((gt.uid == onegt.uid)
			&& (gt.sensorType == onegt.sensorType)
			&& (gt.visibleStartTime == onegt.visibleStartTime))
			return;
	}
	recorder.update(onegt);
}

GTClassInfo::GTClassInfo(char c)
{
	switch (c)
	{
	case 'H':
	case 'h':
		*this = GT_HUMAN;
		break;
	case 'V':
	case 'v':
		*this = GT_VEHICLE;
		break;
	case 'B':
	case 'b':
		*this = GT_BOX;
		break;
	case 'O':
	case 'o':
		*this = GT_OBS;
		break;
	default:
		*this = GT_HUMAN;
	}
}
