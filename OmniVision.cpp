#include "OmniVision.h"

bool OmniVision::init()
{
	string vcCalibFilename;

	cv::FileStorage fs("config.yml", cv::FileStorage::READ);

	std::string stringTimeTmp;//������longlong��ΪFileStorage�����
	fs["startTime"] >> stringTimeTmp;// _startTime;
	fs["refineStep"] >> _refineStep;
	fs["refineUID"] >> _refineUID;
	fs["refineObjClass"] >> _refineObjClass;
	fs["responseTime"] >> _waitKeyTime;
	fs.release();

	std::istringstream is(stringTimeTmp);
	is >> _startTime;

	_priSensors.push_back(&_velodyne);

	if(_refineStep == 1)//ֻ����ϸ��עͼ���ÿһ֡ͼ�񶼶�
		_priSensors.push_back(&_flea2);
	else
		_subSensors.push_back(&_flea2);

	for (auto s : _priSensors)
	{
		if (!s->init("config.yml"))
			return false;
		if (!s->grabNextData())//���������ȶ�ȡһ֡����
			return false;
	}

	//_subSensors.push_back(&_urg);
	_subSensors.push_back(&_gps);
	if (_refineStep == 0)//ֻ���ڴֱ�ע��Ҫ
		_subSensors.push_back(&_ladybug);
	for (auto s : _subSensors)
		if (!s->init("config.yml"))
			return false;

#if isLabel
	if (!_gtLabeler.init())
		return false;
#endif

	_VW_pano.open("pano.avi", CV_FOURCC('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_panoWidth, _panoHeight));
	_VW_mono.open("mono.avi", CV_FOURCC('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_monoWidth, _monoHeight));
	_VW_bv.open("localmap.avi", CV_FOURCC('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_mapSize, _mapSize));

#if !isCalib
	cv::namedWindow(winNamePano, 0);//ͼ��̫��
#endif

	//Ҫ�����е�������������ʼ֮����ܿ�ʼ
	for (auto s : _priSensors)
	{
		long long t;
		if (s->getTime(t))
		{
			if (t > _startTime)
				_startTime = t;
		}
	}

	return true;
}

bool OmniVision::getData()
{
	int activePriSensorID;//��ǰʱ������µ���������
	activePriSensorID = 0;//��ǰʱ������µ���������
	if (!_priSensors[activePriSensorID]->getTime(_currentTime))//��ȡ0�Ŵ������ĵ�ǰʱ��
		return false;//���������ֱ��ȥ����...
	for (int i = 1 ; i < _priSensors.size(); i++)
	{
		long long t;
		if (_priSensors[i]->getTime(t))
		{
			if (t < _currentTime)//�õ��������Ǹ�������id
			{
				_currentTime = t;
				activePriSensorID = i;
			}
		}
	}

	if (!_priSensors[activePriSensorID]->grabNextData())//�����Ĵ�������ȡһ֡������
		return false;

	while (_currentTime < _startTime)
	{
		std::cout << _currentTime << std::endl;
		return getData();//�ݹ�����ָ��ʱ��
	}

	for (auto s : _subSensors)
		if (!s->grabData(_currentTime))
			return false;

	_veloData = _velodyne.getCurrentData();
	std::cout << _veloData.frameNum;
	std::cout << " | velo: " << _veloData.timestamp;

	_monoData = _flea2.getCurrentData();
	std::cout << " | mono: " << _monoData.timestamp;

	_gpsPrevData = _gps.getPrevData();
	_gpsData = _gps.getCurrentData();
	std::cout << " | gps: " << _gpsData.timestamp;

	//_urgData = _urg.getCurrentData();
	//std::cout << " | urg: " << _urgData.timestamp;
	
	_panoData = _ladybug.getCurrentData();
	std::cout << " | pano: " << _panoData.timestamp;
	
	std::cout << std::endl;

	return true;
}

void OmniVision::showLMS()
{
	cv::Mat canvas_pano;
	cv::Mat canvas_mono;
	cv::Mat canvas_bv(cv::Size(_mapSize, _mapSize), CV_8UC3);

	cv::cvtColor(_monoData.img, canvas_mono, CV_BGR2GRAY);
	cv::cvtColor(canvas_mono, canvas_mono, CV_GRAY2BGR);
	cv::cvtColor(_ladybug.getCurrentData().img, canvas_pano, CV_BGR2GRAY);
	cv::cvtColor(canvas_pano, canvas_pano, CV_GRAY2BGR);
	canvas_bv.setTo(0);

	for (auto vp : _urgData.pts)
	{
		cv::Vec3b color = _colormap.getColor(vp.x, -5, 5); //�߶�
		cv::Point2i lp;
		_ladybug.VehicleP2ImageP(vp, lp);
		if (isInImage(lp, canvas_pano))
			canvas_pano.at<cv::Vec3b>(lp) = color; //cv::Vec3b(255, 255, 255);// vp.color

		cv::Point2i fp;
		_flea2.VehicleP2ImageP(vp, fp);
		if (isInImage(fp, canvas_mono))
			circle(canvas_mono, fp, 2, color, 2);

		cv::Point2i bvp;
		bvp.x = vp.x / _pixelSize + _mapSize / 2;
		bvp.y = _mapSize / 2 - vp.y / _pixelSize;
		if (isInImage(bvp, canvas_bv))
			circle(canvas_bv, bvp, 2, color, 2);
	}

	imshow("lms on bv", canvas_bv);
	imshow("lms on mono", canvas_mono);
	imshow("lms on pano", canvas_pano);
}

void OmniVision::testImageP2VehicleP()
{
	_bv_velo.create(cv::Size(_mapSize, _mapSize), CV_8UC3);
	_bv_velo.setTo(0);
	for (int i = 0; i < _bv_velo.rows; i++)
	{
		for (int j = 0; j < _bv_velo.cols; j++)
		{
			cv::Point3d bvp;
			cv::Point2i imgp;
			bvp.x = (j - _mapSize / 2)*_pixelSize;
			bvp.y = (_mapSize / 2 - i)*_pixelSize;
			bvp.z = 0;
			_flea2.VehicleP2ImageP(bvp, imgp);
			if(imgp.x >=0 && imgp.y >= 0 && imgp.x < _monoData.img.cols && imgp.y < _monoData.img.rows)
				_bv_velo.at<cv::Vec3b>(i, j) = _monoData.img.at<cv::Vec3b>(imgp);
		}
	}
	cv::imshow("test_0", _bv_velo);
	_mono_velo.create(_monoData.img.size(), CV_8UC3);
	_mono_velo.setTo(0);
	for (int i = 0; i < _mono_velo.rows; i ++)
	{
		for (int j = 0; j < _mono_velo.cols; j++)
		{
			cv::Point2i imgp(j, i);
			cv::Point3d bvp;
			_flea2.ImageP2VehicleP(imgp, bvp);
			int x = bvp.x / _pixelSize + _mapSize / 2;
			int y = _mapSize / 2 - bvp.y / _pixelSize;
			if (x >= 0 && y >= 0 && x < _bv_velo.cols && y < _bv_velo.rows)
				_mono_velo.at<cv::Vec3b>(imgp) = _bv_velo.at<cv::Vec3b>(y, x);
		}
	}
	cv::imshow("test_1", _mono_velo);
	cv::imshow("test_2", _monoData.img);
	cv::waitKey();
}

void OmniVision::showVelo()
{
	bool isColorIn;
	cv::FileStorage fs("config.yml", cv::FileStorage::READ);
	fs["veloColor"] >> isColorIn;
	fs["minValidVeloHeight"] >> minValidVeloHeight;
	fs["maxValidVeloHeight"] >> maxValidVeloHeight;
	fs.release();

	_bv_velo.create(cv::Size(_mapSize, _mapSize), CV_8UC3);
	_bv_velo.setTo(0);

	cv::cvtColor(_monoData.img, _mono_velo, CV_BGR2GRAY);
	cv::cvtColor(_mono_velo, _mono_velo, CV_GRAY2BGR);
	if (_refineStep == 0)//ֻ���ڴֱ�ע��Ҫ
	{
		cv::cvtColor(_ladybug.getCurrentData().img, _pano_velo, CV_BGR2GRAY);
		cv::cvtColor(_pano_velo, _pano_velo, CV_GRAY2BGR);
	}
	/*for (int y = 0; y < canvas_bv.rows; y++)
	{
		for (int x = 0; x < canvas_bv.cols; x++)
		{
			cv::Point3d vpGround;
			cv::Point2i lp;
			vpGround.x = (x - _mapSize / 2) * _pixelSize;
			vpGround.y = (_mapSize / 2 - y) * _pixelSize;//��λm
			vpGround.z = -0.0;// -0.7;
			_ladybug.VehicleP2ImageP(vpGround, lp);
			if (isInImage(lp, canvas_pano))
				canvas_bv.at<cv::Vec3b>(y, x) = _ladybug.getCurrentData().img.at<cv::Vec3b>(lp);
		}
	}
	cv::imshow("clean bv", canvas_bv);*/
	cv::Rect visROI = cv::Rect(0, 0, _pano_velo.cols, _pano_velo.rows);
	//for (auto vp : _veloData.point)
	//{
	for(int i = 0 ; i < _veloData.point.size(); i ++)
	{
		auto vp = _veloData.point[i];
		//if (vp.z < minValidVeloHeight || vp.z > maxValidVeloHeight)
		//	continue;
#if !isCalib
		cv::Vec3b color;
		//cv::Vec3b color = _colormap.getColor(vp.z, minValidVeloHeight, maxValidVeloHeight); //�߶�
		//cv::Vec3b color = _colormap.getColor(vp.z, 0, 1); //�߶�


		if(isColorIn == 1)
			color = _colormap.getColor(_veloData.intensi[i], 0, 50); //������
		else
			color = _colormap.getColor(vp.z, minValidVeloHeight, maxValidVeloHeight); //�߶�
#endif
	    //cv::Vec3b color = _colormap.getColor(std::sqrtf(vp.z*vp.z + vp.y*vp.y + vp.x*vp.x), 1, 15);// ����

		cv::Point2i lp;
		_ladybug.VehicleP2ImageP(vp, lp);
#if isCalib
		//cv::Vec3b color = _colormap.getColor(lp.x, 0, 2048);
		//visROI = cv::Rect(1260 * ResizeLadybugRadio, 0 * ResizeLadybugRadio, 340 * ResizeLadybugRadio, 250 * ResizeLadybugRadio);
		visROI = cv::Rect(1265 * ResizeLadybugRadio, 0 * ResizeLadybugRadio, 320 * ResizeLadybugRadio, 256 * ResizeLadybugRadio);
		cv::Vec3b color = _colormap.getColor(lp.x, visROI.x, visROI.x + visROI.width);

		if (lp.x > visROI.x+ visROI.width || lp.x < visROI.x || lp.y < visROI.y || lp.y > visROI.y + visROI.height || vp.z < 0.1)
			continue;//�궨��Ŀ��
#endif
		if (_refineStep == 0)//ֻ���ڴֱ�ע��Ҫ
		{
			if (isInImage(lp, _pano_velo))
				_pano_velo.at<cv::Vec3b>(lp) = color; //cv::Vec3b(255, 255, 255);// vp.color
		}

		cv::Point2i bvp(_mapSize / 2 + vp.x / _pixelSize, _mapSize / 2 - vp.y / _pixelSize);
		if (isInImage(bvp, _bv_velo))
			if(_bv_velo.at<cv::Vec3b>(bvp) == cv::Vec3b(0,0,0))//ȡ��ߵ�
				_bv_velo.at<cv::Vec3b>(bvp) = color;

		cv::Point2i fp;
		_flea2.VehicleP2ImageP(vp, fp);
		if (isInImage(fp, _mono_velo))
			_mono_velo.at<cv::Vec3b>(fp) = color;
	}

	if (!_gtLabeler.isTrackerInit())//������ʱ�����...
	{
		cv::imshow(winNameMono, _mono_velo);
		cv::imshow(winNameBV, _bv_velo);
		if (_refineStep == 0)//ֻ���ڴֱ�ע��Ҫ
			cv::imshow(winNamePano, _pano_velo(visROI));
	}

	_VW_mono << _mono_velo;
	if (_refineStep == 0)//ֻ���ڴֱ�ע��Ҫ
		_VW_pano << _pano_velo;
	_VW_bv << _bv_velo;
}

bool OmniVision::generateOneGT_withPanoROI(OneGroundTruth& onegt)
{
	//= getRoughPosPano(gtLabel.getTRect(), _veloData, _ladybug);
	accurateBBox abbox;
	getAccuratePosPano(_gtLabeler.getTRect(), _veloData, _ladybug, abbox);
	//�м�ľ�ֵ���в��ȶ��ɷ֣�����ĵ��ǲ��ǻ��ȶ�Щ��
	//cv::Point3d objPos = abbox.centerMean;
	cv::Point3d objPos = abbox.nearMost;


	if (isSensorTypeMatched(onegt, 'C'))
	{
		onegt.visibleStartTime = _flea2.getCurrentData().timestamp;
		onegt.visibleEndTime = _flea2.getCurrentData().timestamp;
	}
	else if(isSensorTypeMatched(onegt, 'L'))
	{
		onegt.visibleStartTime = _velodyne.getCurrentData().timestamp;
		onegt.visibleEndTime = _velodyne.getCurrentData().timestamp + oneVeloFrameVisibleTimeDuration;//��ʱ��ʩ
	}

	onegt.objYaw = _gtLabeler.getLocalYaw(_gpsData);
	onegt.objClass = _gtLabeler._curInfo.objClass;

	generate2DBBox(_gtLabeler._curInfo, abbox.nearMost, onegt.objYaw, onegt.sensorType, _flea2, onegt.objCorners, _gtLabeler._verification, _monoData.img);
	//��һ���ж��Ƿ���ͼ��ɼ���Χ��
	if (isSensorTypeMatched(onegt, 'C')) 
	{
		if (abs(onegt.objCorners[0].y - onegt.objCorners[2].y) < imageVisibleHeight)
		{
			std::cout << _gtLabeler.getCurUID() << " gt too small\n";
			return false;
		}
		if (onegt.objCorners[0].x < 0)
		{
			if ((onegt.objCorners[2].x - 0) <= (onegt.objCorners[2].x - onegt.objCorners[0].x) / 2)//
			{
				std::cout << _gtLabeler.getCurUID() << " gt 50% out of image(left)\n";
				return false;
			}
			//���ﰴ������Ӧ�ð���ͼ���С�и�һ�£���Ҫ����ͼ���Ե
			onegt.objCorners[0].x = 0;
			onegt.objCorners[3].x = 0;
		}
		if (onegt.objCorners[2].x >= _monoData.img.cols)
		{
			if ((_monoData.img.cols - 1 - onegt.objCorners[0].x) <= (onegt.objCorners[2].x - onegt.objCorners[0].x) / 2)//
			{
				std::cout << _gtLabeler.getCurUID() << " gt 50% out of image(right)\n";
				return false;
			}
			//���ﰴ������Ӧ�ð���ͼ���С�и�һ�£���Ҫ����ͼ���Ե
			onegt.objCorners[1].x = _monoData.img.cols - 1;
			onegt.objCorners[2].x = _monoData.img.cols - 1;
		}
	}
	/*//��һ���ж��Ƿ��ڼ���ɼ���Χ��
	else if (isSensorTypeMatched(onegt, 'L')
		&& sqrt(objPos.x * objPos.x + objPos.y * objPos.y) > bvVisibleDistance)
	{
		std::cout << _gtLabeler.getCurUID() << " gt too small\n";
		return false;
	}//*/

	_gps.VehicleP2GlobalP(objPos, onegt.objPos);
	onegt.uid = _gtLabeler.getCurUID();

	return true;
}

bool OmniVision::generateOneGT_withMonoROI(OneGroundTruth& onegt)
{
	if (isSensorTypeMatched(onegt, 'C'))
	{
		onegt.visibleStartTime = _flea2.getCurrentData().timestamp;
		onegt.visibleEndTime = _flea2.getCurrentData().timestamp;
	}
	else if (isSensorTypeMatched(onegt, 'L'))
	{
		onegt.visibleStartTime = _velodyne.getCurrentData().timestamp;
		onegt.visibleEndTime = _velodyne.getCurrentData().timestamp + oneVeloFrameVisibleTimeDuration;//��ʱ��ʩ
	}

	onegt.objYaw = _gtLabeler.getLocalYaw(_gpsData);
	onegt.objClass = _gtLabeler._curInfo.objClass;

	cv::Rect trackedROI = _gtLabeler.getTRect();
	onegt.objCorners[0] = cv::Point2d(trackedROI.x, trackedROI.y);
	onegt.objCorners[1] = cv::Point2d(trackedROI.x + trackedROI.width, trackedROI.y);
	onegt.objCorners[2] = cv::Point2d(trackedROI.x + trackedROI.width, trackedROI.y + trackedROI.height);
	onegt.objCorners[3] = cv::Point2d(trackedROI.x, trackedROI.y + trackedROI.height);

	//��һ���ж��Ƿ���ͼ��ɼ���Χ��
	if (isSensorTypeMatched(onegt, 'C'))
	{
		if (abs(onegt.objCorners[0].y - onegt.objCorners[2].y) < imageVisibleHeight)
		{
			std::cout << _gtLabeler.getCurUID() << " gt too small\n";
			return false;
		}
		if (onegt.objCorners[0].x < 0)
		{
			if ((onegt.objCorners[2].x - 0) <= (onegt.objCorners[2].x - onegt.objCorners[0].x) / 2)//
			{
				std::cout << _gtLabeler.getCurUID() << " gt 50% out of image(left)\n";
				return false;
			}
			//���ﰴ������Ӧ�ð���ͼ���С�и�һ�£���Ҫ����ͼ���Ե
			onegt.objCorners[0].x = 0;
			onegt.objCorners[3].x = 0;
		}
		if (onegt.objCorners[2].x >= _monoData.img.cols)
		{
			if ((_monoData.img.cols - 1 - onegt.objCorners[0].x) <= (onegt.objCorners[2].x - onegt.objCorners[0].x) / 2)//
			{
				std::cout << _gtLabeler.getCurUID() << " gt 50% out of image(right)\n";
				return false;
			}
			//���ﰴ������Ӧ�ð���ͼ���С�и�һ�£���Ҫ����ͼ���Ե
			onegt.objCorners[1].x = _monoData.img.cols - 1;
			onegt.objCorners[2].x = _monoData.img.cols - 1;
		}
	}
	/*
	//��һ���ж��Ƿ��ڼ���ɼ���Χ��
	else if ((onegt.sensorType == 'L' || onegt.sensorType == 'l')
		&& sqrt(objPos.x * objPos.x + objPos.y * objPos.y) > bvVisibleDistance)
	{
		std::cout << gtLabel.getCurUID() << " gt too small\n";
		return false;
	}

	_gps.VehicleP2GlobalP(objPos, onegt.objPos);*/
	onegt.objPos = cv::Point2d(0, 0);
	onegt.uid = _gtLabeler.getCurUID();

	return true;
}

bool OmniVision::generateOneGT_withBVPoint(OneGroundTruth & onegt)
{
	// todo

	//curInfo xml ���� ���ȡ��ֵ�����ĳһ֡
	//localYaw -> ��ȡ��ֵ�����ĳһ֡ rect[0] rect[1] ������
	//abbox 
	onegt.sensorType = 'L';
	//generate2DBBox(curInfo, abbox, localYaw, onegt.sensorType, flea2, onegt.objCorners);

	return false;
}

void OmniVision::showSavedLabel()
{
#if isLabel
	_gtLabeler.visualizeSavedGT(_flea2, _gps, _monoData, _veloData, _mono_velo, _bv_velo);
#endif

}
void OmniVision::label()
{
#if isLabel
	_gtLabeler.labelSingleFrame(_ladybug, _flea2, _gpsData, _veloData, _pano_velo, _mono_velo, _bv_velo);

	OneGroundTruth onegt;

	if (_gtLabeler.isTrackerInit())
	{
		//����ͼ��������ֵ
		onegt.sensorType = 'C';
		if (generateOneGT_withPanoROI(onegt))
			_gtLabeler.addNewGT(onegt);

		//���ɼ���������ֵ
		onegt.sensorType = 'L';
		if (generateOneGT_withPanoROI(onegt))
			_gtLabeler.addNewGT(onegt);
	}
#endif

}

void OmniVision::addMissingGlobalBVLabel()
{
	if (!_gtLabeler.getRefineSavedGT_timeDuration(_refineUID, _currentTime))
		return;
	_gtLabeler.addMissingGT_bv(_gps, _flea2, _veloData, _refineUID, GTClassInfo(_refineObjClass[0]));
	cv::waitKey(10);
}

void OmniVision::refineGlobalBVLabel()
{
	if (!_gtLabeler.getRefineSavedGT_timeDuration(_refineUID, _currentTime))
		return;
	//���ӻ����һ��ê��ת������ǰ�ֲ�����ϵ��λ��
	_gtLabeler.visualiseLastArchor(_bv_velo, _gps);
	char key = cv::waitKey(_waitKeyTime);
	switch (key)
	{
	case 'A':
	case 'a':
		_gtLabeler.refineSavedGT_bv_addArchor(_gps, _veloData, _refineUID);
		break;
	case 'R':
	case 'r':
		_gtLabeler.refineSavedGT_bv_addFakeArchor(_gps, _veloData, _refineUID, _bv_velo, _flea2);
		break;
	case 27:
		_gtLabeler.refineSavedGT_bv_startSailing(_gps, _refineUID);
		break;
	default:
		break;
		//��ĳ���ط��޸Ŀ�ʼ����ʱ�䣿
			//��������refineSavedGT_bv_startSailing�����ж��Ƿ񳬹�һ����Χ��
			//�������޸Ĵֱ궨��ȥ�����벻�Ե���ֵ

	}

	// gaosq2
	//OneGroundTruth onegt;
	//onegt.sensorType = 'L';
	//if (generateOneGT_withBVPoint('L', uid, veloData.timestamp, index)) ���©��
	//{
	//	_gtLabeler.addNewGT(onegt);
	//}

}

void OmniVision::refineMonoLabel()
{
	bool isAddNewGT = false;
	_gtLabeler._curInfo = GTClassInfo(_refineObjClass[0]);
	_gtLabeler.refineSavedGT_mono(_flea2, _gps, _monoData, _veloData, _mono_velo, _bv_velo, isAddNewGT, _refineUID);

	if (isAddNewGT)
	{
		OneGroundTruth onegt;
		onegt.sensorType = 'C';
		if (generateOneGT_withMonoROI(onegt))
		{
			_gtLabeler.addNewGT(onegt);
		}
	}
}

/*
void OmniVision::showMap()
{
	//velo �� ����궨��û��������

	cv::Mat localMap(cv::Size(_mapSize, _mapSize), CV_64FC1);
	cv::Mat canvas_gm(cv::Size(_mapSize, _mapSize), CV_8UC3);
	cv::Mat localMask(cv::Size(_mapSize, _mapSize), CV_8UC1);
	localMap.setTo(-3.0);
	localMask.setTo(0);
	canvas_gm.setTo(0);
	for (auto vp : _veloData.point)
	{
		if (_mapSize / 2 - vp.y / _pixelSize >= 0 && _mapSize / 2 - vp.y / _pixelSize < _mapSize &&
			_mapSize / 2 + vp.x / _pixelSize >= 0 && _mapSize / 2 + vp.x / _pixelSize < _mapSize)
		{
			double curH = localMap.at<double>(localMap.cols / 2 + vp.x / _pixelSize, localMap.rows / 2 + vp.y / _pixelSize);
			localMap.at<double>(localMap.cols / 2 + vp.x / _pixelSize, localMap.rows / 2 + vp.y / _pixelSize) = curH > vp.z ? curH : vp.z;
			localMask.at<char>(localMap.cols / 2 + vp.x / _pixelSize, localMap.rows / 2 + vp.y / _pixelSize) = 255;
			canvas_gm.at<cv::Vec3b>(localMap.cols / 2 + vp.x / _pixelSize, localMap.rows / 2 + vp.y / _pixelSize)
				= _colormap.getColor(localMap.at<double>(localMap.cols / 2 + vp.x / _pixelSize, localMap.rows / 2 + vp.y / _pixelSize), -3, 3); //�߶�
		}
	}
	cv::imshow("local velo", canvas_gm);

	if (_veloData.frameNum == 0)
		_gpsPrevData = _gpsData;
	double yawDiff = _gpsPrevData.yaw - _gpsData.yaw;//Ҫ��һ��
	int xDiff = (_gpsData.x - _gpsPrevData.x) / _pixelSize;
	int yDiff = (_gpsData.y - _gpsPrevData.y) / _pixelSize;

	//cv::Mat buffer;
	//localMap.copyTo(buffer);
	cv::Point2d centrePt(localMap.cols / 2.0, localMap.rows / 2.0);
	cv::Mat rMat = getRotationMatrix2D(centrePt, yawDiff * 180.0 / CV_PI, 1);
	warpAffine(_globalMap, _globalMap, rMat, _globalMap.size());
	//warpAffine(mask, mask, rMat, mask.size());

	//addWeighted(buffer, 0.1, _globalMap.map(global2local), 0.9, 0, _globalMap.map(global2local));
	//��ô���µ�ͼ��ܳ�

	canvas_gm.setTo(0);
	double updateRate = 0.2;
	//0.0 ~ 0.5
	for (int i = 0; i < _globalMap.rows; i++)//ƽ��
	{
		for (int j = 0; j < _globalMap.cols; j++)//ƽ��
		{
			if (localMask.at<char>(i, j) != 0)//��Ϊ���������û�㵽��������ϵ�ϣ��߶Ȳ���
			{
				if (i + yDiff < 0 || i + yDiff >= _globalMap.rows
					|| j - xDiff < 0 || j - xDiff >= _globalMap.cols)
					continue;
				_globalMap.at<double>(i, j) = updateRate * localMap.at<double>(i, j) + (1 - updateRate) * _globalMap.at<double>(i + yDiff, j - xDiff);
			}
			canvas_gm.at<cv::Vec3b>(i, j) = _colormap.getColor(_globalMap.at<double>(i, j), -3, 3); //�߶�
		}
	}
	cv::imshow("global velo", canvas_gm);

	std::cout << _gpsData.yaw * 180 / CV_PI << ' ' << _gpsData.x << ' ' << _gpsData.y << std::endl;
}*/

void OmniVision::keyborad()
{
#if isLabel
	return;
#endif
	char keyboard = cv::waitKey(10);
	if (keyboard == 27)
		return;
	else if (keyboard == 20)
		cv::waitKey();
	else if (keyboard == 's' || keyboard == 'S')
		ModifyManually();
}

void OmniVision::release()
{
	_VW_bv.release();
	_VW_pano.release();
	_VW_mono.release();
}