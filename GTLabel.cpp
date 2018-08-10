#include "GTLabel.h"

bool GTLabel::init()
{
	cv::FileStorage fs("config.yml", cv::FileStorage::READ);  
	if (!fs.isOpened())
		return false;
	fs["gtFile"] >> outputFile;
	fs["responseTime"] >> _responseTime;
	fs.release();


	extraOutputFile.open(outputFile + std::string(".extra.csv"), std::ios::app);
	if (!extraOutputFile.is_open())
		return false;
	extraOutputFile << "ResizeLadybugRadio: " << ResizeLadybugRadio << std::endl;

	auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	struct tm* ptm = localtime(&tt);
	char timestr[60] = { 0 };
	sprintf(timestr, "%d-%02d-%02d-%02d-%02d-%02d",
		(int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
		(int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
	extraOutputFile << "Time: " << timestr << std::endl;

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
	switch (key)
	{
	case 'r':
	case 'R':
		labeler.label(tracker.roi, monoData.img, winNameMono);
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
		//break; 修改了就顺道存了吧
	case 27:
		recorder.save(outputFile);
		break;
	case 'd':
	case 'D':
		isDeleted = recorder.erase('C', uid, monoData.timestamp);
		recorder.save(outputFile);
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
		cv::Point2d globalPos = (*_mapArchorGPSPositions.rbegin()).second;
		gps.GlobalP2VehicleP(globalPos, vehicleP);

		cv::Point2d bvp;
		bvp.x = vehicleP.x / _pixelSize + _mapSize / 2;
		bvp.y = _mapSize / 2 - vehicleP.y / _pixelSize;

		cv::circle(canvas, bvp, 5, CV_RGB(0, 0, 255));
		cv::imshow(winNameBV, canvas);
		//cv::line(canvas, bvp[3], bvp[0], CV_RGB(255, 0, 0));
		/*std::stringstream ssTmp;
		ssTmp.str("");
		ssTmp << gt.uid << ": " << gt.objClass;
		cv::putText(canvas_bv, ssTmp.str(), cv::Point2i(bvp[0].x, bvp[0].y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);*/
	}
}

cv::Point2d getInterpolatedPosition(long long time, long long st, long long et, cv::Point2d sp, cv::Point2d ep)
{
	if (time < st)
		return sp;
	if (time > et)
		return ep;
	double factor = double(time - st) / double(et - st);
	std::cout << factor << std::endl;
	cv::Point2d buffer;
	buffer.x = (1 - factor) * sp.x + factor * ep.x;
	buffer.y = (1 - factor) * sp.y + factor * ep.y;
	return buffer;
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
		_mapArchorGPSPositions.insert(std::make_pair(veloData.timestamp, recorder.gts[index].objPos));
	}
}

void GTLabel::refineSavedGT_bv_addFakeArchor(GPSReader & gps, VelodyneData & veloData, int uid, cv::Mat& canvas_bv, Flea2Reader& flea2)
{
	int index;
	if (recorder.getOneSavedGTIndex('L', uid, veloData.timestamp, index))
	{
		OneGroundTruth gt = recorder.gts[index];

		double dx, dy;
		//c1是x-y+(左上方)，c2是x+y+(右上方)
		dx = gt.objCorners[2].x - gt.objCorners[1].x;
		dy = gt.objCorners[2].y - gt.objCorners[1].y;
		double yawOfThisGT = atan2(dy, dx);// [-pi,+pi]
		gt.objYaw = getLocalYaw(gps.getCurrentData());

		cv::Rect roi;
		double yaw;
		fang::selectROI(roi, yaw, gt, canvas_bv, flea2, gps);
		
		_mapArchorGPSPositions.insert(std::make_pair(veloData.timestamp, gt.objPos));

		//朝向插值太麻烦...我不想做...必须要做
	}
}

void GTLabel::modifyOneSavedGT(OneGroundTruth& gt, cv::Point2d refinedCenter, GPSReader& gps)
{
	if (!gps.grabData(gt.visibleStartTime))
		system("pause");
	if (isSensorTypeMatched(gt, 'C'))
	{
		gt.objPos = refinedCenter;
	}
	else
	{
		gt.objPos = refinedCenter;
		//计算四个角点					
		cv::Point3d vehicleCorners[4];
		cv::Point2d globalCorners[4];
		cv::Point2d globalCenter(0.0, 0.0);
		for (int i = 0; i < 4; i++)
		{
			vehicleCorners[i].x = gt.objCorners[i].x;
			vehicleCorners[i].y = gt.objCorners[i].y;
			vehicleCorners[i].z = 0;
			gps.VehicleP2GlobalP(vehicleCorners[i], globalCorners[i]);
			globalCenter += globalCorners[i];
		}
		globalCenter.x /= 4;
		globalCenter.y /= 4;

		cv::Point2d shift = refinedCenter - globalCenter;
		for (int i = 0; i < 4; i++)
		{
			globalCorners[i] += shift;
			gps.GlobalP2VehicleP(globalCorners[i], vehicleCorners[i]);
			gt.objCorners[i].x = vehicleCorners[i].x;
			gt.objCorners[i].y = vehicleCorners[i].y;
		}
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
			cv::Point2d refinedCenter;
			if (_mapArchorGPSPositions.size() == 1)
			{
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
					refinedCenter = getInterpolatedPosition(gt.visibleStartTime, (*prevE).first, (*nextE).first, (*prevE).second, (*nextE).second);
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
			double dx, dy;
			
			//c1是x-y+(左上方)，c2是x+y+(右上方)
			dx = recorder.gts[index].objCorners[2].x - recorder.gts[index].objCorners[1].x;
			dy = recorder.gts[index].objCorners[2].y - recorder.gts[index].objCorners[1].y;

			double yawOfThisGT = atan2(dy, dx);// [-pi,+pi]
			
			setGlobalYaw(yawOfThisGT, gps.getCurrentData());//实时更新globalYaw
		}
	}
	else if (time > _refineEndTime)
	{
		recorder.save(outputFile);
		exit(0);
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
		cv::Point2i bvp[4];
		for (auto gt : saved_gt)
		{
#if reGenerateCorners
			gps.GlobalP2VehicleP(gt.objPos, objPos);
			curInfo = GTClassInfo(gt.objClass);
			generate2DBBox(objPos, gt.sensorType, flea2, gt.objCorners);
#endif
			for (int i = 0; i < 4; i++)
			{
				bvp[i].x = gt.objCorners[i].x / _pixelSize + _mapSize / 2;
				bvp[i].y = _mapSize / 2 - gt.objCorners[i].y / _pixelSize;
			}
			cv::line(canvas_bv, bvp[0], bvp[1], CV_RGB(255, 0, 0));
			cv::line(canvas_bv, bvp[1], bvp[2], CV_RGB(255, 0, 0));
			cv::line(canvas_bv, bvp[2], bvp[3], CV_RGB(255, 0, 0));
			cv::line(canvas_bv, bvp[3], bvp[0], CV_RGB(255, 0, 0));
			std::stringstream ssTmp;
			ssTmp.str("");
			ssTmp << gt.uid << ": " << gt.objClass;
			cv::putText(canvas_bv, ssTmp.str(), cv::Point2i(bvp[0].x, bvp[0].y + textHeight), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);
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
	putText(canvas_pano, TrackingStatusNames[lastStatus], cv::Point(4, 20 * 2), cv::FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);
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
		cv::Point2i bvp[4];
		for (int i = 0; i < 4; i++)
		{
			bvp[i].x = onegt.objCorners[i].x / _pixelSize + _mapSize / 2;
			bvp[i].y = _mapSize / 2 - onegt.objCorners[i].y / _pixelSize;
		}
		cv::line(canvas_bv, bvp[0], bvp[1], CV_RGB(255, 0, 0));
		cv::line(canvas_bv, bvp[1], bvp[2], CV_RGB(255, 0, 0));
		cv::line(canvas_bv, bvp[2], bvp[3], CV_RGB(255, 0, 0));
		cv::line(canvas_bv, bvp[3], bvp[0], CV_RGB(255, 0, 0));
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
		tracker.uid++;
		curMaxUID = tracker.uid;
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
		recorder.save(outputFile);
		break;
	}

	if (tracker_flag == TS_TRACKING)
	{
		extraOutputFile << ladybug.getCurrentData().timestamp << ','
			<< tracker.uid << ','
			<< tracker.roi.x << ',' << tracker.roi.y << ','
			<< tracker.roi.width << ',' << tracker.roi.height << std::endl;
	}
}

void GTLabel::addNewGT(OneGroundTruth onegt)
{
	for (auto gt : recorder.gts)//效率有点低
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
