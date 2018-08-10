#include "../../header.h"

#include "../../LadybugPGR/LoadLadyBugPgr.h"
#include "../../Flea2/flea2reader.h"
#include "../../Velodyne/velodynedatainterface.h"
#include "../../GPS/gps.h"

#include "../GTClassSizeInfo/GTClassSizeInfo.h"
#include "../Recorder/Recorder.h"
#include "../../getVeloPosFromImage.h"
#include "../../misc.h"

using cv::Rect;
using cv::Mat;
using cv::Point2f;
using cv::line;


#define rotate_step CV_PI/18.0



class Labeler
{
public:
	void labelMore(Rect& roi, double& yaw, LadybugReader& ladybug, Flea2Reader& flea2, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv);
	void label(Rect& roi, Mat img, std::string winName);
};

namespace fang
{
	class ROISelector {
	public:
		Rect select(Mat img, bool fromCenter = true);
		Rect select(const cv::String& windowName, Mat img, bool showCrossair = true, bool fromCenter = true);

		void select(Rect& roi, double& yaw, LadybugReader& ladybug, Flea2Reader& flea2, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv);
		void select(Rect& roi, double& yaw, OneGroundTruth& gt, cv::Mat& canvas_bv, Flea2Reader& flea2, GPSReader& gps);

		void select(const cv::String& windowName, Mat img, std::vector<Rect> & boundingBox, bool fromCenter = true);

		struct handlerT {
			// basic parameters
			bool isDrawing;
			Rect box;
			Mat image;

			// extra parameters for bv

			//输入的时候应该按照如下两个信息初始化？
			//cv::Point2d bvCenter_bvCoord;//真实坐标
			//float angle_bvCoord;

			Mat canvas_bv;
			Mat canvas_mono;

			float yaw;
			//cv::Point2f bvCenter_imgCoord;//用于生成RotatedRect的
			//cv::Size2f bvSize;
			//cv::RotatedRect bvBox;

			// parameters for drawing from the center
			bool drawFromCenter;
			Point2f center;

			// initializer list
			handlerT() : isDrawing(false), drawFromCenter(true) {};
		}selectorParams;

		// to store the tracked objects
		std::vector<handlerT> objects;

	private:
		static void mouseHandler(int event, int x, int y, int flags, void *param);
		void opencv_mouse_callback(int event, int x, int y, int, void *param);

		static void mouseHandler_bv(int event, int x, int y, int flags, void *param);
		void opencv_mouse_callback_bv(int event, int x, int y, int, void *param);

		// save the keypressed characted
		int key;
	};

	Rect selectROI(Mat img, bool fromCenter = true);
	Rect selectROI(const cv::String& windowName, Mat img, bool showCrossair = true, bool fromCenter = true);
	void selectROI(const cv::String& windowName, Mat img, std::vector<Rect> & boundingBox, bool fromCenter = true);
	void selectROI(Rect& roi, double& yaw, LadybugReader& ladybug, Flea2Reader& flea2, VelodyneData& veloData, cv::Mat& canvas_pano, cv::Mat& canvas_mono, cv::Mat& canvas_bv);
	void selectROI(Rect& roi, double& yaw, OneGroundTruth& gt, cv::Mat& canvas_bv, Flea2Reader& flea2, GPSReader& gps);
}