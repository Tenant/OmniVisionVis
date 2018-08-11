#include "GTClassSizeInfo.h"

cv::Rect MatchingMethod(cv::Mat img, cv::Mat templ)
{
	/// 将被显示的原图像
	Mat img_display;
	img.copyTo(img_display);

	/// 创建输出结果的矩阵
	int result_cols = img.cols - templ.cols + 1;
	int result_rows = img.rows - templ.rows + 1;

	cv::Mat result(result_cols, result_rows, CV_32FC1);

	/// 进行匹配和标准化
	int match_method = CV_TM_CCOEFF_NORMED;
	matchTemplate(img, templ, result, match_method);
	//normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());

	/// 通过函数 minMaxLoc 定位最匹配的位置
	double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
	cv::Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

	/// 对于方法 SQDIFF 和 SQDIFF_NORMED, 越小的数值代表更高的匹配结果. 而对于其他方法, 数值越大匹配越好
	if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
	{
		matchLoc = minLoc;
	}
	else
	{
		matchLoc = maxLoc;
	}

	/// 让我看看您的最终结果
	float score = result.at<float>(matchLoc);
	//std::cout << "score:" << score << std::endl;
	rectangle(img_display, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar::all(0), 2, 8, 0);
	rectangle(result, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar::all(0), 2, 8, 0);

	cv::imshow("resulted target", img_display(cv::Rect(matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows))));
	//imshow("image_window", img_display);
	//imshow("result_window", result);
	if(score > MIN_MATCHING_SCORE)
		return cv::Rect(matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows));
	return cv::Rect(0, 0, 0, 0);
}

void generate2DBBox(GTClassInfo info, cv::Point3d objCenterPos, double localYaw, char sensorType, Flea2Reader& flea2, cv::Point2d cornerp[4], cv::Mat templ, cv::Mat img)
{
	cv::Point3d vp[4];
	cv::Point2i fp[4];
	cv::Point2d bvLocalP[4];

	double rot_angle = localYaw;
	double oWidth = info.width;
	double oHeight = info.height;
	double oLength = info.length;

	if (isSensorTypeMatched(sensorType, 'C'))
	{
		//中间的均值点有不稳定成分，最近的点是不是还稳定些？
		//vp[0] = abbox.centerMean;//顺时针
		//vp[0] = abbox.nearMost;//顺时针
		vp[0] = objCenterPos;
		vp[0].x -= oWidth / 2;
		vp[0].z -= oHeight / 2;
		vp[1] = vp[0];
		vp[1].x += oWidth;
		vp[2] = vp[1];
		vp[2].z += oHeight;
		vp[3] = vp[2];
		vp[3].x -= oWidth;
		for (int i = 0; i < 4; i++)
		{
			flea2.VehicleP2ImageP(vp[i], fp[i]);
			cornerp[i] = fp[i];
		}
		int maxX = cornerp[0].x;
		int minX = cornerp[0].x;
		int maxY = cornerp[0].y;
		int minY = cornerp[0].y;
		for (int i = 1; i < 4; i++)
		{
			if (cornerp[i].x < minX)
				minX = cornerp[i].x;
			if (cornerp[i].x > maxX)
				maxX = cornerp[i].x;
			if (cornerp[i].y < minY)
				minY = cornerp[i].y;
			if (cornerp[i].y > maxY)
				maxY = cornerp[i].y;
		}
		//摄像头真值
		//1.坐标点有些不是矩形（有几条差一两个像素）
		//2.坐标不是左上角顺时针，而是右上角顺时针
		cornerp[0] = cv::Point2d(minX, minY);
		cornerp[1] = cv::Point2d(maxX, minY);
		cornerp[2] = cv::Point2d(maxX, maxY);
		cornerp[3] = cv::Point2d(minX, maxY);
		//外面用到[0].y-[2].y，所以就算不排序也要保证这里合理；还用到了[0][3]左，[1][2]右

		cv::Point2i monoTop;
		cv::Point2i monoBottom;
		//cv::Point3d head3d = abbox.centerMean; //中间的均值点有不稳定成分，最近的点是不是还稳定些？
		cv::Point3d head3d = objCenterPos;
		head3d.z += oHeight / 2;
		//cv::Point3d feet3d = abbox.centerMean;
		cv::Point3d feet3d = objCenterPos;
		feet3d.z -= oHeight / 2;
		//其实高度是固定的，feet设为0就好
		//std::cout << "head3d.z:" << head3d.z << " ,feet3d.z:" << feet3d.z << std::endl;
		flea2.VehicleP2ImageP(head3d, monoTop);
		flea2.VehicleP2ImageP(feet3d, monoBottom);
		int heightInMono = monoBottom.y - monoTop.y;


		//外面用到[0].y-[2].y，所以就算不排序也要保证这里合理；还用到了[0][3]左，[1][2]右

		if (img.data != NULL && templ.data != NULL)
		{
			//先确定这样可行
			double resizeRatio = (double)heightInMono / (double)templ.rows;//这个和物体位置有关，不是定值
			if (resizeRatio*templ.cols < 5 || resizeRatio * templ.rows < 5)
				resizeRatio = 1;
			cv::resize(templ, templ, cv::Size(resizeRatio*templ.cols, resizeRatio*templ.rows));
			cv::cvtColor(img, img, CV_BGR2GRAY);
			cv::cvtColor(templ, templ, CV_BGR2GRAY);
			cv::Rect enlargedRegion = cv::Rect(cornerp[0], cornerp[2]);

			if (isValidROI(enlargedRegion, img.size()))
			//可能解算出来的框本身就不对了	//那就不放大，按照不对的输出
			{
				cv::Mat targetImage = img(cutValidROI(enlargedRegion, img.size()));
				if (targetImage.cols > 10 && targetImage.rows > 10)
					cv::imshow("original target image", targetImage);
				cv::imshow("original tracked image", templ);

				enlargedRegion.x -= enlargedRegion.width * 2.5;
				enlargedRegion.y -= enlargedRegion.height;
				enlargedRegion.width *= 6;
				enlargedRegion.height *= 3;
				enlargedRegion = cutValidROI(enlargedRegion, img.size());
				if (enlargedRegion.width < templ.cols || enlargedRegion.height < templ.rows || enlargedRegion.x < 0 || enlargedRegion.y < 0)
				{
					cornerp[0] = cv::Point2d(0, 0);
					cornerp[1] = cv::Point2d(0, 0);
					cornerp[2] = cv::Point2d(0, 0);
					cornerp[3] = cv::Point2d(0, 0);
				}
				else
				{
					cv::Rect searchRect = MatchingMethod(img(enlargedRegion), templ);
					searchRect.x += enlargedRegion.x;
					searchRect.y += enlargedRegion.y;
					cornerp[0] = cv::Point2d(searchRect.x, searchRect.y);
					cornerp[1] = cv::Point2d(searchRect.x + searchRect.width, searchRect.y);
					cornerp[2] = cv::Point2d(searchRect.x + searchRect.width, searchRect.y + searchRect.height);
					cornerp[3] = cv::Point2d(searchRect.x, searchRect.y + searchRect.height);
				}
			}
			else
			{
				cornerp[0] = cv::Point2d(0, 0);
				cornerp[1] = cv::Point2d(0, 0);
				cornerp[2] = cv::Point2d(0, 0);
				cornerp[3] = cv::Point2d(0, 0);
			}
		}

		//下面的注释记得打开
		/*
		for (int i = 1; i < 4; i++)
		{
			if (cornerp[i].x < minX)
				minX = cornerp[i].x;
			if (cornerp[i].x > maxX)
				maxX = cornerp[i].x;
			if (cornerp[i].y < minY)
				minY = cornerp[i].y;
			if (cornerp[i].y > maxY)
				maxY = cornerp[i].y;
		}
		//摄像头真值
		//1.坐标点有些不是矩形（有几条差一两个像素）
		//2.坐标不是左上角顺时针，而是右上角顺时针
		cornerp[0] = cv::Point2d(minX, minY);
		cornerp[1] = cv::Point2d(maxX, minY);
		cornerp[2] = cv::Point2d(maxX, maxY);
		cornerp[3] = cv::Point2d(minX, maxY);*/
	}
	else if (isSensorTypeMatched(sensorType, 'L'))
	{
		bvLocalP[0] = cv::Point2d(-oWidth / 2, -oLength / 2);
		bvLocalP[1] = cv::Point2d(-oWidth / 2, oLength / 2);
		bvLocalP[2] = cv::Point2d(oWidth / 2, oLength / 2);
		bvLocalP[3] = cv::Point2d(oWidth / 2, -oLength / 2);
		for (int i = 0; i < 4; i++)
		{
			rotatePoint2d(bvLocalP[i], rot_angle);
			//中间的均值点有不稳定成分，最近的点是不是还稳定些？
			//bvLocalP[i].x += abbox.centerMean.x;
			//bvLocalP[i].y += abbox.centerMean.y;
			bvLocalP[i].x += objCenterPos.x;
			bvLocalP[i].y += objCenterPos.y;

			cornerp[i] = bvLocalP[i];
		}
	}
}