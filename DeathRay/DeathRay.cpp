// DeathRay.cpp : Defines the entry point for the console application.
//

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
#include "LaserTracking.h"
#include "Phidget.h"
#include "ServoCtrl.h"
#include <dlib/svm.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	//init camera 
	VideoCapture cap(0); //capture the video from webcam
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	PhidgetController pctrl;
	ServoCtrl ctrl(pctrl, 2, 3);

	LaserTracking laser(cap, ctrl, true);
	laser.Track();

	//laser.Calibrate(ctrl, 10);
	//laser.ShootingRange(ctrl, false, false);
	/* laser.Calibrate(ctrl, 20);
	laser.ShootingRange(ctrl, false, false);*/
	//ctrl.MoveToServoPos(100, 100);
	//int result = laser.Track(true, true);
	cap.release();
	return 0;
}

//Uncomment and run this case you will be facing problems with the initial - this will free up the components
//int main(int argc,  char* argv[])
//{
//	PhidgetController pctrl;
//	ServoCtrl ctrl(pctrl, 2, 3);
//	return 0;
//}