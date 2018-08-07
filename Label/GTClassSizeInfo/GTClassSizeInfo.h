#pragma once
#include <string>
#include "../../header.h"
#include "../../Flea2/flea2reader.h"
#include "../../getVeloPosFromImage.h"
#include "../Recorder/Recorder.h"

using cv::Mat;

class GTClassInfo
{
public:
	GTClassInfo() : name("UNKNOWN"), objClass(' '), width(0), length(0), height(0) {};
	GTClassInfo(std::string _n, double _w, double _l, double _h) : name(_n), objClass(_n[0]), width(_w), length(_l), height(_h) {};
	GTClassInfo(char c);

	char objClass;
	std::string name;
	double width;
	double length;
	double height;
	//double yaw;
};

const GTClassInfo GT_HUMAN = GTClassInfo("HUMAN", 0.7, 0.7, 1.7);
const GTClassInfo GT_VEHICLE = GTClassInfo("VEHICLE", 2.0, 4.8, 2.0);
const GTClassInfo GT_BOX = GTClassInfo("BOX", 1.0, 1.0, 1.0);
const GTClassInfo GT_OBS = GTClassInfo("OBS", 0.5, 0.5, 0.5);

#define MIN_MATCHING_SCORE 0.4
//小于这个得分就返回一个空的rect

void generate2DBBox(GTClassInfo info, accurateBBox abbox, double localYaw, char sensorType, Flea2Reader& flea2, cv::Point2d cornerp[4], cv::Mat templ = cv::Mat(), cv::Mat img = cv::Mat());