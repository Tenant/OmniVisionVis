#include "Labeler.h"

void Labeler::label(Rect& roi, Mat img, std::string winName = winNamePano)
{
	std::string labelWindowName = winName;
	//	cv::namedWindow(labelWindowName);
	roi = fang::selectROI(labelWindowName, img);
	//	cv::destroyWindow(labelWindowName);
	//	while
}

void Labeler::labelMore(Rect& roi, double& yaw, LadybugReader& ladybug, Flea2Reader& flea2, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv)
{
	fang::selectROI(roi, yaw, ladybug, flea2, veloData, canvas_pano, canvas_mono, canvas_bv);
}

namespace fang
{
	void ROISelector::mouseHandler(int event, int x, int y, int flags, void *param){
		ROISelector *self = static_cast<ROISelector*>(param);
		self->opencv_mouse_callback(event, x, y, flags, param);
	}

	void ROISelector::opencv_mouse_callback(int event, int x, int y, int flag, void *param){
		handlerT * data = (handlerT*)param;
		switch (event){
			// update the selected bounding box
		case cv::EVENT_MOUSEMOVE:
			if (data->isDrawing){
				if (data->drawFromCenter){
					data->box.width = 2 * (x - data->center.x)/*data->box.x*/;
					data->box.height = 2 * (y - data->center.y)/*data->box.y*/;
					data->box.x = data->center.x - data->box.width / 2.0;
					data->box.y = data->center.y - data->box.height / 2.0;
				}
				else{
					data->box.width = x - data->box.x;
					data->box.height = y - data->box.y;
				}
			}
			break;

			// start to select the bounding box
		case cv::EVENT_LBUTTONDOWN:
			data->isDrawing = true;
			data->box = cvRect(x, y, 0, 0);
			data->center = Point2f((float)x, (float)y);
			break;

			// cleaning up the selected bounding box
		case cv::EVENT_LBUTTONUP:
			data->isDrawing = false;
			if (data->box.width < 0){
				data->box.x += data->box.width;
				data->box.width *= -1;
			}
			if (data->box.height < 0){
				data->box.y += data->box.height;
				data->box.height *= -1;
			}
			break;

		case cv::EVENT_MOUSEWHEEL: //opencv2 doesn't has this definition
			double resizeRatio = 0.025;
			cv::Point2i shiftLength(data->box.width * resizeRatio / 2, data->box.height * resizeRatio / 2);
			if (data->box.width * resizeRatio / 2 < 1.0 && data->box.width <= data->box.height)//假如框已经小到小于1，而且要判断更小的那个
			{
				shiftLength.x = 1;
				shiftLength.y = data->box.height / data->box.width;//(data->box.height * resizeRatio / 2) / data->box.width * resizeRatio / 2
			}
			else if (data->box.height * resizeRatio / 2 < 1.0 && data->box.width >= data->box.height)
			{
				shiftLength.x = data->box.width / data->box.height;
				shiftLength.y = 1;
			}
			if (flag > 0)//zoom in
			{
				data->box.x += shiftLength.x;
				data->box.y += shiftLength.y;
				data->box.width -= shiftLength.x * 2;
				data->box.height -= shiftLength.y * 2;
			}
			else//zoom out
			{
				data->box.x -= shiftLength.x;
				data->box.y -= shiftLength.y;
				data->box.width += shiftLength.x * 2;
				data->box.height += shiftLength.y * 2;
			}
			break;
		}
	}

	Rect ROISelector::select(Mat img, bool fromCenter){
		return select("ROI selector", img, fromCenter);
	}

	Rect ROISelector::select(const cv::String& windowName, Mat img, bool showCrossair, bool fromCenter){
		
		key = 0;

		//初始化自己的参数
		/*selectorParams.bvCenter_imgCoord;//这个位置改一点图像是不会变的...
		selectorParams.bvSize = ;
		selectorParams.angle_imgCoord = 0;
		selectorParams.bvBox = cv::RotatedRect(selectorParams.bvCenter_imgCoord, selectorParams.bvSize, selectorParams.angle_imgCoord);
		*/

		// set the drawing mode
		selectorParams.drawFromCenter = fromCenter;

		// show the image and give feedback to user
		imshow(windowName, img);

		// copy the data, rectangle should be drawn in the fresh image
		selectorParams.image = img.clone();

		// end selection process on SPACE (32) or ENTER (13)
		while (!(key == 32 || key == 13)){
			// select the object
			setMouseCallback(windowName, mouseHandler, (void *)&selectorParams);

			// fine tuning bbox
			switch (key)
			{
			case 'a':
			case 'A':
				selectorParams.box.x--;
				break;
			case 'd':
			case 'D':
				selectorParams.box.x++;
				break;
			case 'w':
			case 'W':
				selectorParams.box.y--;
				break;
			case 's':
			case 'S':
				selectorParams.box.y++;
				break;
			case 'q':
			case 'Q':
				//selectorParams.angle_bvCoord -= 
			default:
				break;
			}

			if (selectorParams.box.x < 0)
			{
				selectorParams.box.x = 0;
			}
			if (selectorParams.box.y < 0)
			{
				selectorParams.box.y = 0;
			}
			if (selectorParams.box.x >= selectorParams.image.cols - selectorParams.box.width)
			{
				int newx = selectorParams.image.cols - 1 - selectorParams.box.width;
				if (newx >= 0)
				{
					selectorParams.box.x = selectorParams.image.cols - 1 - selectorParams.box.width;
				}
				else
				{ 
					selectorParams.box.width = selectorParams.image.cols - 1 - selectorParams.box.x;
				}
			}
			if (selectorParams.box.y >= selectorParams.image.rows - selectorParams.box.height)
			{
				int newy = selectorParams.image.rows - 1 - selectorParams.box.height;
				if (newy >= 0)
				{
					selectorParams.box.y = selectorParams.image.rows - 1 - selectorParams.box.height;
				}
				else
				{
					selectorParams.box.height = selectorParams.image.rows - 1 - selectorParams.box.y;
				}
			}

			// draw the selected object
			rectangle(
				selectorParams.image,
				selectorParams.box,
				cv::Scalar(255, 0, 0), 2, 1
				);

			// draw cross air in the middle of bounding box
			if (showCrossair){
				// horizontal line
				line(
					selectorParams.image,
					cv::Point((int)selectorParams.box.x, (int)(selectorParams.box.y + selectorParams.box.height / 2)),
					cv::Point((int)(selectorParams.box.x + selectorParams.box.width), (int)(selectorParams.box.y + selectorParams.box.height / 2)),
					cv::Scalar(255, 0, 0), 2, 1
					);

				// vertical line
				line(
					selectorParams.image,
					cv::Point((int)(selectorParams.box.x + selectorParams.box.width / 2), (int)selectorParams.box.y),
					cv::Point((int)(selectorParams.box.x + selectorParams.box.width / 2), (int)(selectorParams.box.y + selectorParams.box.height)),
					cv::Scalar(255, 0, 0), 2, 1
					);
			}

			// show the image bouding box
			imshow(windowName, selectorParams.image);

			// reset the image
			selectorParams.image = img.clone();

			//get keyboard event, extract lower 8 bits for scancode comparison
			key = cv::waitKey(1) & 0xFF;
		}
		//上面画的和最后用到的其实很可能不是一个，要加旋转


		return selectorParams.box;
	}

	void ROISelector::select(const cv::String& windowName, Mat img, std::vector<Rect> & boundingBox, bool fromCenter){
		std::vector<Rect> box;
		Rect temp;
		key = 0;

		// show notice to user
		printf("Select an object to track and then press SPACE or ENTER button!\n");
		printf("Finish the selection process by pressing ESC button!\n");

		// while key is not ESC (27)
		for (;;) {
			temp = select(windowName, img, true, fromCenter);
			if (key == 27) break;
			if (temp.width > 0 && temp.height > 0)
				box.push_back(temp);
		}
		boundingBox = box;
	}

	void ROISelector::select(Rect& roi, double& yaw, LadybugReader& ladybug, Flea2Reader& flea2, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv)
	{

		key = 0;

		//初始化自己的参数
		/*selectorParams.bvCenter_imgCoord;//这个位置改一点图像是不会变的...
		selectorParams.bvSize = ;
		selectorParams.angle_imgCoord = 0;
		selectorParams.bvBox = cv::RotatedRect(selectorParams.bvCenter_imgCoord, selectorParams.bvSize, selectorParams.angle_imgCoord);
		*/

		// set the drawing mode
		selectorParams.drawFromCenter = true;

		// show the image and give feedback to user
		imshow(winNamePano, canvas_pano);

		// copy the data, rectangle should be drawn in the fresh image
		selectorParams.image = canvas_pano.clone();

		cv::Rect visualROI;
		// end selection process on SPACE (32) or ENTER (13) //第一次先把图像放大
		while (!(key == 'R' || key == 'r')) {
			cv::setMouseCallback(winNamePano, mouseHandler, (void *)&selectorParams);
			if (selectorParams.box.x < 0)
			{
				selectorParams.box.x = 0;
			}
			if (selectorParams.box.y < 0)
			{
				selectorParams.box.y = 0;
			}
			if (selectorParams.box.x >= selectorParams.image.cols - selectorParams.box.width)
			{
				int newx = selectorParams.image.cols - 1 - selectorParams.box.width;
				if (newx >= 0)
				{
					selectorParams.box.x = selectorParams.image.cols - 1 - selectorParams.box.width;
				}
				else
				{
					selectorParams.box.width = selectorParams.image.cols - 1 - selectorParams.box.x;
				}
			}
			if (selectorParams.box.y >= selectorParams.image.rows - selectorParams.box.height)
			{
				int newy = selectorParams.image.rows - 1 - selectorParams.box.height;
				if (newy >= 0)
				{
					selectorParams.box.y = selectorParams.image.rows - 1 - selectorParams.box.height;
				}
				else
				{
					selectorParams.box.height = selectorParams.image.rows - 1 - selectorParams.box.y;
				}
			}

			// draw the selected object
			rectangle(
				selectorParams.image,
				selectorParams.box,
				cv::Scalar(255, 0, 0), 2, 1
			);

			// show the image bouding box
			cv::imshow(winNamePano, selectorParams.image);

			// reset the image
			selectorParams.image = canvas_pano.clone();
			selectorParams.canvas_mono = canvas_mono.clone();
			selectorParams.canvas_bv = canvas_bv.clone();

			//get keyboard event, extract lower 8 bits for scancode comparison
			key = cv::waitKey(1) & 0xFF;
		}
		visualROI = cutValidROI(selectorParams.box, canvas_pano.size());
		// show the image and give feedback to user
		imshow(winNamePano, canvas_pano(visualROI));
		cv::waitKey();
		// copy the data, rectangle should be drawn in the fresh image
		selectorParams.image = canvas_pano(visualROI).clone();

		//自定义变量的初始化
		selectorParams.yaw = 0;
		if(yaw > 0 && yaw < CV_2PI)
			selectorParams.yaw = yaw;

		// end selection process on SPACE (32) or ENTER (13)
		while (!(key == 32 || key == 13)) {
			// select the object
			cv::setMouseCallback(winNamePano, mouseHandler, (void *)&selectorParams);

			// fine tuning bbox
			switch (key)
			{
			case 'a':
			case 'A':
				selectorParams.box.x--;
				break;
			case 'd':
			case 'D':
				selectorParams.box.x++;
				break;
			case 'w':
			case 'W':
				selectorParams.box.y--;
				break;
			case 's':
			case 'S':
				selectorParams.box.y++;
				break;
			case 'q':
			case 'Q':
				selectorParams.yaw -= rotate_step;
				if (selectorParams.yaw < 0)
					selectorParams.yaw += CV_2PI;
				break;
			case 'e':
			case 'E':
				selectorParams.yaw += rotate_step;
				if (selectorParams.yaw > CV_2PI)
					selectorParams.yaw -= CV_2PI;
				break;
			default:
				break;
			}

			if (selectorParams.box.x < 0)
			{
				selectorParams.box.x = 0;
			}
			if (selectorParams.box.y < 0)
			{
				selectorParams.box.y = 0;
			}
			if (selectorParams.box.x >= selectorParams.image.cols - selectorParams.box.width)
			{
				int newx = selectorParams.image.cols - 1 - selectorParams.box.width;
				if (newx >= 0)
				{
					selectorParams.box.x = selectorParams.image.cols - 1 - selectorParams.box.width;
				}
				else
				{
					selectorParams.box.width = selectorParams.image.cols - 1 - selectorParams.box.x;
				}
			}
			if (selectorParams.box.y >= selectorParams.image.rows - selectorParams.box.height)
			{
				int newy = selectorParams.image.rows - 1 - selectorParams.box.height;
				if (newy >= 0)
				{
					selectorParams.box.y = selectorParams.image.rows - 1 - selectorParams.box.height;
				}
				else
				{
					selectorParams.box.height = selectorParams.image.rows - 1 - selectorParams.box.y;
				}
			}

			// draw the selected object
			rectangle(
				selectorParams.image,
				selectorParams.box,
				cv::Scalar(255, 0, 0), 2, 1
			);

			// draw cross air in the middle of bounding box
			//if (showCrossair) 
			{
				// horizontal line
				cv::line(
					selectorParams.image,
					cv::Point((int)selectorParams.box.x, (int)(selectorParams.box.y + selectorParams.box.height / 2)),
					cv::Point((int)(selectorParams.box.x + selectorParams.box.width), (int)(selectorParams.box.y + selectorParams.box.height / 2)),
					cv::Scalar(255, 0, 0), 2, 1
				);

				// vertical line
				cv::line(
					selectorParams.image,
					cv::Point((int)(selectorParams.box.x + selectorParams.box.width / 2), (int)selectorParams.box.y),
					cv::Point((int)(selectorParams.box.x + selectorParams.box.width / 2), (int)(selectorParams.box.y + selectorParams.box.height)),
					cv::Scalar(255, 0, 0), 2, 1
				);
			}

			///s
			//log
			/*std::stringstream convertor;
			convertor.str("");
			convertor << curPos << " / " << _totalFrameNum << "; "
			<< std::setprecision(4) << 100 * static_cast<double>(curPos) / static_cast<double>(_totalFrameNum) << "%";
			putText(canvas, convertor.str(), Point(4, 20), FONT_HERSHEY_SIMPLEX, 0.55, CV_RGB(255, 0, 0), 2);*/
			if (selectorParams.box.area() > 0)
			{
				GTClassInfo curInfo = GT_VEHICLE;
				double rot_angle = selectorParams.yaw;
				double oWidth = curInfo.width;
				double oHeight = curInfo.height;
				double oLength = curInfo.length;

				//可视化在pano 上面已经做了
				//rectangle(canvas_pano, selectorParams.box, CV_RGB(0, 0, 255));
				//imshow(winNamePano, canvas_pano);

				accurateBBox aBBox;
				cv::Rect realSelectedBox = selectorParams.box;
				realSelectedBox.x += visualROI.x;
				realSelectedBox.y += visualROI.y;
				getAccuratePosPano(realSelectedBox, veloData, ladybug, aBBox);
				
				//可视化在mono， 重复实现了， 单独提取出来变成函数
				OneGroundTruth onegt;
				onegt.sensorType = 'C';
				generate2DBBox(curInfo, aBBox, selectorParams.yaw, onegt.sensorType, flea2, onegt.objCorners);
				onegt.draw(selectorParams.canvas_mono, CV_RGB(255, 0, 0));
				cv::imshow(winNameMono, selectorParams.canvas_mono);

				//可视化在bv， 重复实现了， 单独提取出来变成函数
				onegt.sensorType = 'L';
				generate2DBBox(curInfo, aBBox, selectorParams.yaw, onegt.sensorType, flea2, onegt.objCorners);

				//为了测试计算朝向对不对
				//std::cout << "debug0 :" << rot_angle << std::endl;
				//std::cout << "debug1 :" << onegt.objCorners[1] << "|||" << onegt.objCorners[2] << std::endl;
				//std::cout << "debug2 :" << atan2(onegt.objCorners[2].y - onegt.objCorners[1].y, onegt.objCorners[2].x - onegt.objCorners[1].x) << std::endl;

				cv::Point2i bvp[4];
				for (int i = 0; i < 4; i++)
				{
					bvp[i].x = onegt.objCorners[i].x / _pixelSize + _mapSize / 2;
					bvp[i].y = _mapSize / 2 - onegt.objCorners[i].y / _pixelSize;
				}
				cv::line(selectorParams.canvas_bv, bvp[0], bvp[1], CV_RGB(255, 0, 0));
				cv::line(selectorParams.canvas_bv, bvp[1], bvp[2], CV_RGB(255, 0, 0));
				cv::line(selectorParams.canvas_bv, bvp[2], bvp[3], CV_RGB(255, 0, 0));
				cv::line(selectorParams.canvas_bv, bvp[3], bvp[0], CV_RGB(255, 0, 0));
				cv::imshow(winNameBV, selectorParams.canvas_bv);
				///e
			}

			// show the image bouding box
			cv::imshow(winNamePano, selectorParams.image);

			// reset the image
			selectorParams.image = canvas_pano(visualROI).clone();
			selectorParams.canvas_mono = canvas_mono.clone();
			selectorParams.canvas_bv = canvas_bv.clone();

			//get keyboard event, extract lower 8 bits for scancode comparison
			key = cv::waitKey(1) & 0xFF;
		}
		//上面画的和最后用到的其实很可能不是一个，要加旋转

		roi = selectorParams.box;
		roi.x += visualROI.x;
		roi.y += visualROI.y;
		yaw = selectorParams.yaw;
		//return selectorParams.box;
	}

	ROISelector _selector;
	Rect selectROI(Mat img, bool fromCenter){
		return _selector.select("ROI selector", img, true, fromCenter);
	};

	Rect selectROI(const cv::String& windowName, Mat img, bool showCrossair, bool fromCenter){
//		printf("Select an object to track and then press SPACE or ENTER button!\n");
		return _selector.select(windowName, img, showCrossair, fromCenter);
	};

	void selectROI(Rect& roi, double& yaw, LadybugReader& ladybug, Flea2Reader& flea2, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv)
	{
		_selector.select(roi, yaw, ladybug, flea2, veloData, canvas_pano, canvas_mono, canvas_bv);
	}

	void selectROI(const cv::String& windowName, Mat img, std::vector<Rect> & boundingBox, bool fromCenter){
		return _selector.select(windowName, img, boundingBox, fromCenter);
	}
}
