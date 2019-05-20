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

	cv::FileNode fnTmp = fs["minImageVisibleHeight"];
	if (!fnTmp.isNone())
	{
		fnTmp >> _imageVisibleHeight;
	}

	fs.release();

	std::istringstream is(stringTimeTmp);
	is >> _startTime;

	_priSensors.push_back(&_velodyne);


	for (auto s : _priSensors)
	{
		if (!s->init("config.yml"))
			return false;
		if (!s->grabNextData())//���������ȶ�ȡһ֡����
			return false;
	}


	_subSensors.push_back(&_ladybug);
	_subSensors.push_back(&_mask);
	for (auto s : _subSensors)
		if (!s->init("config.yml"))
			return false;


	//_VW_pano.open("pano.avi", CV_FOURCC('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_panoWidth, _panoHeight));
	//_VW_mono.open("mono.avi", CV_FOURCC('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_monoWidth, _monoHeight));
	_VW_pano.open("pano.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_panoWidth, 512));
	_VW_mono.open("mono.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_panoWidth, 512));
	_VW_bv.open("localmap.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_mapSize, _mapSize));
	_VW_mask.open("maskmap.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), _velodyneFPS, cv::Size(_mapSize, _mapSize));


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
	do
	{
		for (int i = 1; i < _priSensors.size(); i++)
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

		if (!_priSensors[activePriSensorID]->getTime(_currentTime))//��ȡ0�Ŵ������ĵ�ǰʱ��
			return false;//���������ֱ��ȥ����...		
		std::cout << _currentTime << std::endl;
	} while (_currentTime < _startTime);

	//_currentTime -= 820;//���campusԭʼvelodyne pcap���ݵ��޸�
	_currentTime -= 800;//���campusԭʼvelodyne dsvl���ݵ��޸�
	for (auto s : _subSensors)
		if (!s->grabData(_currentTime))
			return false;

	_veloData = _velodyne.getCurrentData();
	std::cout << _veloData.frameNum;
	std::cout << " | velo: " << _veloData.timestamp;

	_maskData = _mask.getCurrentData();
	std::cout << " | mask: " << _maskData.timestamp;


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

	cv::cvtColor(canvas_mono, canvas_mono, cv::COLOR_GRAY2BGR);
	cv::cvtColor(_ladybug.getCurrentData().img, canvas_pano, cv::COLOR_BGR2GRAY);
	cv::cvtColor(canvas_pano, canvas_pano, cv::COLOR_GRAY2BGR);
	canvas_bv.setTo(0);



	imshow("lms on bv", canvas_bv);
	imshow("lms on mono", canvas_mono);
	imshow("lms on pano", canvas_pano);
}

void OmniVision::showVelo()
{
	//for (int i = 0; i < _maskData.bboxes.size(); i++)
	//{
	//	_maskData.bboxes[i].y += 128 - 75;//�����֪��Ϊʲô�����Ǵ��ʱ���bug
	//}

	trackedObjs tObjs;
	tObjs.update(cv::Mat(), _maskData);

	bool isColorIn;
	cv::FileStorage fs("config.yml", cv::FileStorage::READ);
	fs["veloColor"] >> isColorIn;
	fs["minValidVeloHeight"] >> minValidVeloHeight;
	fs["maxValidVeloHeight"] >> maxValidVeloHeight;
	fs.release();

	_bv_velo.create(cv::Size(_mapSize, _mapSize), CV_8UC3);
	_bv_velo.setTo(0);

	cv::Mat _bv_mask;
	_bv_mask.create(cv::Size(_mapSize, _mapSize), CV_8UC3);
	_bv_mask.setTo(0);

	cv::cvtColor(_ladybug.getCurrentData().img, _pano_velo, cv::COLOR_BGR2GRAY);
	cv::cvtColor(_pano_velo, _pano_velo, cv::COLOR_GRAY2BGR);

	///�������ơ�
	//1.��pano��bv��bv_mask�ӽ��¿��ӻ���
	//2.��ÿһ��������DSVL�ָ�飬����ͨ��DSVL�ָ�������������ĵ㣻
	//3.����ÿ��ͼ��ָ���Ӧ��DSVL�ָ�飬����ͨ��ͼ��ָ�������������ļ���ָ�飬�����������弤����������
	for (int i = 0; i < BKNUM_PER_FRM; i ++)
	{
		for (int j = 0; j < PTNUM_PER_BLK; j++)
		{
			auto vp = _veloData.onefrm->dsv[i].points[j];
			//if (vp.z < minValidVeloHeight || vp.z > maxValidVeloHeight)
			//	continue;

			cv::Vec3b color;

			if (isColorIn == 1)
				color = _colormap.getColor(vp.i, 0, 50); //������
			else
				color = _colormap.getColor(vp.z, minValidVeloHeight, maxValidVeloHeight); //�߶�

			cv::Point2i lp;
			cv::Point3d cvvp;
			cvvp.x = vp.x;
			cvvp.y = vp.y;
			cvvp.z = vp.z;
			_ladybug.VehicleP2ImageP(cvvp, lp);
			cv::Point2i bvp(_mapSize / 2 + vp.x / _pixelSize, _mapSize / 2 - vp.y / _pixelSize);

			bool isInPano = isInImage(lp, _pano_velo);
			bool isInBV = isInImage(bvp, _bv_velo);
			if (isInPano)
			{
				//_pano_velo.at<cv::Vec3b>(lp) = color; //cv::Vec3b(255, 255, 255);// vp.color
				cv::circle(_pano_velo, lp, 1, color, -1);
			}
			if (isInBV)
			{
				//if (_bv_velo.at<cv::Vec3b>(bvp) == cv::Vec3b(0, 0, 0))//ȡ��ߵ�
				if (_bv_velo.at<cv::Vec3b>(bvp).val[0] < color[0])//ȡ��ߵ�
					_bv_velo.at<cv::Vec3b>(bvp) = color;

				if (isInPano && _maskData.mask.at<cv::Vec3b>(lp).val[0] != 0)
				{
					int localUID = _maskData.mask.at<cv::Vec3b>(lp).val[0];
					trackedObj tObj = tObjs.getTrack(localUID);//����Ϊ�˻�ȡ��ɫ

					_bv_mask.at<cv::Vec3b>(bvp) = tObj.color;
				}
			}

			//associate all candidate LiDAR Seg with Img Seg
			int lSegLabel = _veloData.onefrm->dsv[i].lab[j];
			if (lSegLabel <= 0)
				continue;
			auto segTmp = _veloData.seg.find(lSegLabel);
			if (segTmp == _veloData.seg.end())
			{
				_veloData.seg.insert({ lSegLabel, lSeg(i, j) });//0������ʱ���Ҫ��size=1
			}
			else
			{
				segTmp->second.blk_id.push_back(i);
				segTmp->second.pt_id.push_back(j);
				segTmp->second.totalPtsNum++;
			}

			if (isInBV && isInPano)//������޸�Ҫ���޸�mask
			{
				if (_maskData.mask.at<cv::Vec3b>(lp).val[0] != 0)
				{
					int localUID = _maskData.mask.at<cv::Vec3b>(lp).val[0];
					tObjs.modifyTrack(localUID, lSegLabel);
				}
/*				else
				{
					for (int k = 0; k < _maskData.bboxes.size(); k++)
					{
						if (isInImage(lp, _maskData.bboxes[k]))
						{
							tObjs.modifyTrack(k, lSegLabel);
						}
					}
				}*/
			}
		}
	}
	

	/*/�������еĵ㣬������bbox�ڵģ���ֻ��bboxû�зָ��Ĵ�����
	int minVeloPointThreshold = 20;
	for (int i = 0; i < BKNUM_PER_FRM; i++)
	{
		for (int j = 0; j < PTNUM_PER_BLK; j++)
		{
			int lSegLabel = _veloData.onefrm->dsv[i].lab[j];
			if (lSegLabel <= 0)
				continue;
			auto segTmp = _veloData.seg.find(lSegLabel);
			if (segTmp->second.totalPtsNum < minVeloPointThreshold)
			{
				//continue;
			}

			auto vp = _veloData.onefrm->dsv[i].points[j];
			cv::Point2i lp;
			cv::Point3d cvvp;
			cvvp.x = vp.x;
			cvvp.y = vp.y;
			cvvp.z = vp.z;
			_ladybug.VehicleP2ImageP(cvvp, lp);
			cv::Point2i bvp(_mapSize / 2 + vp.x / _pixelSize, _mapSize / 2 - vp.y / _pixelSize);

			bool isInPano = isInImage(lp, _pano_velo);
			bool isInBV = isInImage(bvp, _bv_velo);

			if (isInBV && isInPano)//������޸�Ҫ���޸�mask
			{
				if (_maskData.mask.at<cv::Vec3b>(lp).val[0] == 0)
				{
					for (int k = 0; k < _maskData.bboxes.size(); k++)
					{
						if (isInImage(lp, _maskData.bboxes[k]))
						{
							tObjs.modifyTrack(k, lSegLabel);
						}
					}
				}
			}
		}
	}*/

//�㷨����
//1. ��������㣬ͳ��ÿһ�����Ĺ������ܵ�����ͶӰ��ͼ��ȷ��ÿһ��mask�ָ���м�������ָ��
//2. ����ÿ��mask�ָ�飬��������ĳһ������ָ��ĵ��������Ѿ�������ֵ�͹�������
//3. ����ÿ��mask�ָ�飬�����Ӧ����ָ��õ�����ά����

	//����ÿ��mask�ָ��, �����ӻ�
	cv::Mat testMask;
	_panoData.img.copyTo(testMask);
	for (int i = 0; i < _maskData.bboxes.size(); i++)
	{
		//_maskData.bboxes[i].y += 128 - 75;//�����֪��Ϊʲô�����Ǵ��ʱ���bug
		trackedObj tObj = tObjs.getTrack(_maskData.uid_local[i]);
		for (int y = _maskData.bboxes[i].y; y < _maskData.bboxes[i].y + _maskData.bboxes[i].height; y++)
		{
			if (y > testMask.rows - 1)
				continue;
			for (int x = _maskData.bboxes[i].x; x < _maskData.bboxes[i].x + _maskData.bboxes[i].width; x++)
				//for (int x = 0; x < testMask.cols; x++)
			{
				cv::Vec3b maskUID = _maskData.mask.at<cv::Vec3b>(y, x);
				//if (maskUID != cv::Vec3b::all(0))
				//	std::cout << (int)maskUID.val[0] << ' ' << _maskData.uid_local[i] << std::endl;

				if (maskUID == cv::Vec3b::all(_maskData.uid_local[i]))
				{
					//std::cout << (int)maskUID.val[0] << ' ' << tObj.color << std::endl;
					if (y < testMask.rows)
						testMask.at<cv::Vec3b>(y, x) = tObj.color;
				}
			}
		}

		cv::Rect drawRect = _maskData.bboxes[i];

		cv::rectangle(testMask, drawRect, tObj.color);

		cv::putText(testMask, _maskData.clsNames[i] + " " + std::to_string(_maskData.scores[i]), cv::Point(drawRect.x, drawRect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.55, tObj.color, 2);
		//imshow("testMask", testMask);
		//cv::waitKey();
	}
	imshow("testMask", testMask);

	//step2.  ����ÿ��mask�ָ�飬��������ĳһ������ָ��ĵ��������Ѿ�������ֵ�͹�������
	//(�������û�зָ�����Ҫ��������
	double segIoUThreshold = 0.5;//���ڴ���ֵ�ľ�������һ�𣬰�������д���߼������ֵһ��Ҫ����0.5������Ψһmaster
	for (int imgSegIndex = 0; imgSegIndex < tObjs.tracks.size(); imgSegIndex++)
	{
		trackedObj& ti = tObjs.tracks[imgSegIndex];
		int totalAssociatedPtNum = 0;
		for (int i = 0; i < ti.lSegID.size(); i++)
		{
			int key = ti.lSegID[i];
			int imgSeg_lSegSize = ti.lSegID_ptsNum[i];
			int lSegTotalSize = _veloData.seg[key].totalPtsNum;//����û���ж�key�Ƿ����
			
			if (_veloData.seg[key].associatedImgSegIndex != -1) //���統ǰlSeg��û�б�����ĳһ��iSeg
				continue;
			double IoU = double(imgSeg_lSegSize) / double(lSegTotalSize);
			if (IoU > segIoUThreshold)
			{
				totalAssociatedPtNum += imgSeg_lSegSize;
				_veloData.seg[key].associatedImgSegIndex = imgSegIndex;
			}
		}
		std::cout << imgSegIndex << ':' << totalAssociatedPtNum << std::endl;//���ֺܶ���ʵû�зָ��...
	}

	_VW_bv << _bv_velo;
	_VW_mono << testMask;
	cv::imshow("pano", _pano_velo);
	cv::imshow("bv", _bv_velo);
	cv::imshow("bv_mask", _bv_mask);
	cv::waitKey();
	//step3. ����ÿ��DSVL�ָ�飬�����Ӧ����ָ��õ�����ά���������ӻ�
	//���ӻ����...
 	_bv_mask.setTo(0);
	cv::cvtColor(_ladybug.getCurrentData().img, _pano_velo, cv::COLOR_BGR2GRAY);
	cv::cvtColor(_pano_velo, _pano_velo, cv::COLOR_GRAY2BGR);
	for(auto it : _veloData.seg)
	//for(int lSegIndex = 0; lSegIndex < _veloData.seg.size(); lSegIndex++)
	{
		lSeg& lSegIt = it.second;
		if (lSegIt.associatedImgSegIndex == -1)
			continue;
		cv::Vec3b color = tObjs.getTrack(lSegIt.associatedImgSegIndex).color;
		for (int i = 0; i < lSegIt.blk_id.size(); i++)
		{
			point3fi vp = _veloData.onefrm->dsv[lSegIt.blk_id[i]].points[lSegIt.pt_id[i]];
			cv::Point2i lp;
			cv::Point3d cvvp;
			cvvp.x = vp.x;
			cvvp.y = vp.y;
			cvvp.z = vp.z;
			_ladybug.VehicleP2ImageP(cvvp, lp);
			cv::Point2i bvp(_mapSize / 2 + vp.x / _pixelSize, _mapSize / 2 - vp.y / _pixelSize);

			bool isInPano = isInImage(lp, _pano_velo);
			bool isInBV = isInImage(bvp, _bv_velo);
			if (isInPano)
			{
				//_pano_velo.at<cv::Vec3b>(lp) = color; //cv::Vec3b(255, 255, 255);// vp.color
				cv::circle(_pano_velo, lp, 1, color, -1);
			}
			if (isInBV)
			{
				_bv_mask.at<cv::Vec3b>(bvp) = color;
			}
		}
	}
	_VW_pano << _pano_velo;
	_VW_mask << _bv_mask;
	cv::imshow("pano seg", _pano_velo);
	cv::imshow("bv mask-seg", _bv_mask);
	//������ά������
	//...

}

bool OmniVision::keyborad()
{
#if isLabel
	return;
#endif
	char keyboard = cv::waitKey(10);
	if (keyboard == 27)
		return false;
	else if (keyboard == 20)
		cv::waitKey();
	else if (keyboard == 's' || keyboard == 'S')
		ModifyManually();
	return true;
}

void OmniVision::release()
{
	_VW_bv.release();
	_VW_mask.release();
	_VW_pano.release();
	_VW_mono.release();
}