#ifndef _LASERTRACKING_H__
#define _LASERTRACKING_H__
#include "stdafx.h"
#include "ServoCtrl.h"
#include <dlib/svm.h>

using namespace dlib;


typedef matrix<double, 2, 1> sample_type;
typedef radial_basis_kernel<sample_type> kernel_type;


class LaserTracking
{
public:
	LaserTracking(cv::VideoCapture & camera, ServoCtrl &ctrl, bool daylight);
	~LaserTracking();
	int Track(bool showAdjustWindow = false, bool showCameraSettings = false, bool displayThreshold = true, bool displayGrayScale = true);
	int ShootingRange(ServoCtrl &controller, bool showAdjustWindow = false, bool showCameraSettings = false, bool displayThreshold = true, bool displayGrayScale = true);
	int Calibrate(ServoCtrl controller, int nbOfPoints);
	int CalibrateUniform(ServoCtrl controller);
	void Shoot(int x, int y, ServoCtrl controller);
private:
	void printHelp();
	static void onMouse(int event, int x, int y, int, void*);
	ServoCtrl &controller;
	cv::VideoCapture cam;
	static LaserTracking * laser;
	//bool shootted;
	int targetX=-1;
	int targetY=-1;
	int trackingX = -1;
	int trackingY = -1;
	bool day;
	const int DEFAULT_THRESHOLD_DAY = 185;
	const int DEFAULT_THRESHOLD_NIGHT_LOW = 105;
	const int DEFAULT_THRESHOLD_NIGHT_HIGH = 240;
	const string THRESHOLD_WINDOW_NAME = "Threshold Level";
	struct fann *ann;
	decision_function<kernel_type> df;

	//void btCallback(int state, void* userdata);
};

#endif