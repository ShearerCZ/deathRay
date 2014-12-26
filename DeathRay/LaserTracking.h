#pragma once 

#include <opencv2/core/core.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <phidget21.h>
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
	ServoCtrl &controller;
	cv::VideoCapture cam;
	int targetX=-1;
	int targetY=-1;
	int trackingX = -1;
	int trackingY = -1;
	bool day;
	const int DEFAULT_THRESHOLD_DAY = 185;
	const int DEFAULT_THRESHOLD_NIGHT_LOW = 105;
	const int DEFAULT_THRESHOLD_NIGHT_HIGH = 240;
	const std::string THRESHOLD_WINDOW_NAME = "Threshold Level";
	struct fann *ann;
	decision_function<kernel_type> df;
};