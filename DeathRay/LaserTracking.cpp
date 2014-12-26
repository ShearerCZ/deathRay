
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
#include <windows.h>
#include <process.h>
#include <time.h>
#include <fann.h>
#include <iostream>
#include <vector>
#include <dlib/svm.h>

using namespace cv;
using namespace dlib;
using namespace std;

LaserTracking::LaserTracking(cv::VideoCapture &camera, ServoCtrl &ctrl, bool daylight)
	: controller(ctrl)
{
	cam = camera;
	day = daylight;
}


LaserTracking::~LaserTracking()
{
}

void LaserTracking::printHelp()
{
	cout << "  Laser tracking for single laser beam" << endl << "--------------------------------------" << endl;
	cout << "  Press Esc to finish" << endl;
	cout << "  Press Space to clear the path" << endl;
}

void LaserTracking::Shoot(int x, int y, ServoCtrl controller)
{
	cout << "Shooting to camera X: " << x << ", Y: " << y << endl;
	fann_type *calc_out;
	float in[2] = { (float)x / 640.0, (float)y / 480.0 };

	cout << "Scaled input: X: " << in[0] << ", Y: " << in[1] << endl;

	fann_print_connections(ann);
	calc_out = fann_run(ann, in);
	calc_out[0] = calc_out[0] * 70.0 + 80.0;
	calc_out[1] = calc_out[1] * 40.0 + 79.0;



	// here we declare that our samples will be 1 dimensional column vectors.
	typedef matrix<double, 2, 1> sample_type;
	sample_type m;
	m(0, 0) = x;
	m(1, 0) = y;
	double result = df(m);

	cout << "X: " << (int)result / 10000 << ", Y: " << (int)result % 10000 << endl;
	controller.MoveToServoPos((int)result / 10000, (int)result % 10000, false);

	//float xCam = 1.0*x / 640.0;
	//float yCam = 1.0*y / 480.0;
	//controller.MoveToCameraPos(xCam, yCam, true);
	cout << "Shooting to X: " << result / 10000 << ", Y: " << (int)result % 10000 << endl;
}


int LaserTracking::Track(bool showAdjustWindow, bool showCameraSettings, bool displayThreshold, bool displayGrayScale)
{
	int thresholdLevelL = DEFAULT_THRESHOLD_DAY;
	if (!day) thresholdLevelL = DEFAULT_THRESHOLD_NIGHT_LOW;
	int thresholdLevelH = 255;
	if (!day)  thresholdLevelH = DEFAULT_THRESHOLD_NIGHT_HIGH;
	//null check 
	if (!cam.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	printHelp();
	//displays camera settings - not working on Mac
	if (showCameraSettings)
	{
		cam.set(CV_CAP_PROP_SETTINGS, 1);
	}
	//displays the adjust window
	if (showAdjustWindow)
	{
		//create a window for control the accuracy
		namedWindow(THRESHOLD_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
		////Create trackbars in "Accuracy" window
		createTrackbar("Lower Limit", THRESHOLD_WINDOW_NAME, &thresholdLevelL, 255); //grayscale (0 - 255)
		createTrackbar("Higher Limit", THRESHOLD_WINDOW_NAME, &thresholdLevelH, 255); //grayscale (0 - 255)
	}

	//Capture a temporary image from the camera to get the size
	Mat imgTmp;
	cam.read(imgTmp);
	imshow("Camera View", imgTmp);
	int iLastX = -1;
	int iLastY = -1;
	//Create a black image with the size as the camera output to see the laser tracking
	Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);
	//Starts loop until Escape key is pressed
	while (true)
	{
		//original image
		Mat imgOriginal;
		bool bSuccess = cam.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			return -1;
		}
		//convert to grayscale
		Mat imgbGrayScale;
		cvtColor(imgOriginal, imgbGrayScale, COLOR_BGR2GRAY);
		//gets the thresholded image
		Mat imgThresholded;
		inRange(imgbGrayScale, Scalar(thresholdLevelL), Scalar(thresholdLevelH), imgThresholded);

		//morphological opening (removes small objects)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));
		//morphological opening (removes holes)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));


		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		//
		if (dArea >= 100 && dArea < 10000)
		{
			//calculate the position
			int x = (dM10 / dArea);
			int y = (dM01 / dArea);
			cout << "Laser beam pointing to X: " << x << ", Y: " << y << endl;
			//if last is different draw a new line
			if (iLastX >= 0 && iLastY >= 0 && x >= 0 && y >= 0)
			{
				//Draw a red line from the previous point to the current point
				line(imgLines, Point(x, y), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
			}

			iLastX = x;
			iLastY = y;
			//draw yellow rectactangle on original image
			cv::rectangle(imgOriginal,
				Point(x - 5, y - 5),
				Point(x + 5, y + 5),
				Scalar(80, 240, 255),
				2,
				8);
		}
		//show the thresholded image
		if (displayThreshold) imshow("Thresholded Image", imgThresholded);
		if (displayGrayScale) imshow("GrayScale", imgbGrayScale);
		imgOriginal = imgOriginal + imgLines;
		imshow("Camera View", imgOriginal);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			destroyWindow("Camera View");
			if (showAdjustWindow) destroyWindow(THRESHOLD_WINDOW_NAME);
			if (displayThreshold) destroyWindow("Thresholded Image");
			if (displayGrayScale)  destroyWindow("GrayScale");
			break;
		}
		if (waitKey(30) == 32) //wait for space to clear
		{
			imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);
			cout << "Clearing the path" << endl;
		}
	}
	return 0;
}


int LaserTracking::ShootingRange(ServoCtrl &controller, bool showAdjustWindow, bool showCameraSettings, bool displayThreshold, bool displayGrayScale)
{
	int thresholdLevelL = DEFAULT_THRESHOLD_DAY;
	if (!day) thresholdLevelL = DEFAULT_THRESHOLD_NIGHT_LOW;
	int thresholdLevelH = 255;
	if (!day)  thresholdLevelH = DEFAULT_THRESHOLD_NIGHT_HIGH;
	//null check 
	if (!cam.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	printHelp();
	//displays camera settings - not working on Mac
	if (showCameraSettings)
	{
		cam.set(CV_CAP_PROP_SETTINGS, 1);
	}
	//displays the adjust window
	if (showAdjustWindow)
	{
		//create a window for control the accuracy
		namedWindow(THRESHOLD_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
		////Create trackbars in "Accuracy" window
		createTrackbar("Lower Limit", THRESHOLD_WINDOW_NAME, &thresholdLevelL, 255); //grayscale (0 - 255)
		createTrackbar("Higher Limit", THRESHOLD_WINDOW_NAME, &thresholdLevelH, 255); //grayscale (0 - 255)
	}

	int previousX = -1;
	int	previousY = -1;
	//Capture a temporary image from the camera to get the size
	Mat imgTmp;
	cam.read(imgTmp);
	imshow("Camera View", imgTmp);
	// setting callback for shooting - current object is passed as pointer into the class
	setMouseCallback("Camera View", [](int event, int x, int y, int flags, void* userData) {
		LaserTracking* laser = static_cast<LaserTracking*>(userData);
		laser->trackingX = x;
		laser->trackingY = y;

		if (event == EVENT_LBUTTONDOWN && laser->targetX != x && laser->targetY != y)
		{
			laser->targetX = x;
			laser->targetY = y;
			laser->Shoot(x, y, laser->controller);
		}
		cout << "mouse at X: " << x << ", Y: " << y << endl;
	}, this);

	//Starts loop until Escape key is pressed
	while (true)
	{
		//original image
		Mat imgOriginal;
		bool bSuccess = cam.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			return -1;
		}
		//convert to grayscale
		Mat imgbGrayScale;
		cvtColor(imgOriginal, imgbGrayScale, COLOR_BGR2GRAY);
		//gets the thresholded image
		Mat imgThresholded;
		inRange(imgbGrayScale, Scalar(thresholdLevelL), Scalar(thresholdLevelH), imgThresholded);

		//morphological opening (removes small objects)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));
		//morphological opening (removes holes)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));


		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		//
		if (dArea >= 100 && dArea < 2000)
		{
			//calculate the position
			int x = (dM10 / dArea);
			int y = (dM01 / dArea);
			cout << "Laser beam pointing to X: " << x << ", Y: " << y << endl;
			//if last is different send calibration info
			if (previousX >= 0 && previousY >= 0 && x == previousX && y == previousY)
			{
				controller.CalibrateCameraPos(1.0*x / 640, 1.0*y / 480);
			}

			previousX = x;
			previousY = y;
			//draw yellow rectactangle on original image
			cv::rectangle(imgOriginal,
				Point(x - 5, y - 5),
				Point(x + 5, y + 5),
				Scalar(80, 240, 255),
				2,
				8);
		}
		if (trackingX >= 0 && trackingY >= 0)
		{
			cv::rectangle(imgOriginal,
				Point(trackingX - 8, trackingY - 8),
				Point(trackingX + 8, trackingY + 8),
				Scalar(240, 0, 255),
				2,
				8);

		}

		if (targetX >= 0 && targetY >= 0)
		{
			cv::rectangle(imgOriginal,
				Point(targetX - 5, targetY - 5),
				Point(targetX + 5, targetY + 5),
				Scalar(80, 0, 255),
				5,
				8);

		}
		//show the thresholded image
		if (displayThreshold) imshow("Thresholded Image", imgThresholded);
		if (displayGrayScale) imshow("GrayScale", imgbGrayScale);
		imshow("Camera View", imgOriginal);
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			destroyWindow("Camera View");
			if (showAdjustWindow) destroyWindow(THRESHOLD_WINDOW_NAME);
			if (displayThreshold) destroyWindow("Thresholded Image");
			if (displayGrayScale)  destroyWindow("GrayScale");
			break;
		}
		if (waitKey(30) == 32) //wait for space to clear the path
		{
			trackingX = -1;
			trackingY = -1;
			targetX = -1;
			targetY = -1;
			cout << "Clearing the path" << endl;
		}
	}

	return 0;
}

int LaserTracking::Calibrate(ServoCtrl controller, int nbOfPoints)
{
	if (nbOfPoints <= 1)
	{
		// if pi=oints out of range setup it
		nbOfPoints = 2;
	}
	int thresholdLevelL = DEFAULT_THRESHOLD_DAY;
	if (!day) thresholdLevelL = DEFAULT_THRESHOLD_NIGHT_LOW;
	int thresholdLevelH = 255;
	if (!day)  thresholdLevelH = DEFAULT_THRESHOLD_NIGHT_HIGH;
	//null check 
	if (!cam.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	cout << "  Laser calibration for single laser beam" << endl << "--------------------------------------" << endl;

	//displays the adjust window
	//create a window for control the accuracy
	//namedWindow(THRESHOLD_WINDOW_NAME, CV_GUI_EXPANDED);
	//////Create trackbars in "Accuracy" window
	//createTrackbar("Lower Limit", THRESHOLD_WINDOW_NAME, &thresholdLevelL, 255); //grayscale (0 - 255)
	//createTrackbar("Higher Limit", THRESHOLD_WINDOW_NAME, &thresholdLevelH, 255); //grayscale (0 - 255)
	int previousX = -1;
	int	previousY = -1;
	srand(time(NULL));
	double servoPosX = 100, servoPosY = 100;


	//FANN initialization BEGIN
	fann_type *calc_out;
	const unsigned int num_input = 2;
	const unsigned int num_output = 2;
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 3;

	printf("Creating network.\n");
	ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);

	//fann_set_activation_steepness_hidden(ann, 0.5);
	//fann_set_activation_steepness_output(ann, 0.5);

	fann_set_activation_function_hidden(ann, FANN_GAUSSIAN);
	fann_set_activation_function_output(ann, FANN_GAUSSIAN);

	fann_set_training_algorithm(ann, FANN_TRAIN_INCREMENTAL);
	//fann_set_learning_momentum(ann, 0.3);
	//FANN initialization END


	// here we declare that our samples will be 1 dimensional column vectors.
	typedef matrix<double, 2, 1> sample_type;



	// now we are making a typedef for the kind of kernel we want to use.  i picked the

	// radial basis kernel because it only has one parameter and generally gives good

	// results without much fiddling.

	typedef radial_basis_kernel<sample_type> kernel_type;





	std::vector<sample_type> samples;

	std::vector<double> targets;




	for (int i = 0; i < nbOfPoints; i++)
	{
		if (i == 0)
		{
			servoPosX = 100;
			servoPosY = 100;
			controller.MoveToServoPos(servoPosX, servoPosY, true);
		}
		else
		{
			servoPosX = 80 + std::rand() % 70;
			servoPosY = 79 + std::rand() % 40;
			controller.MoveToServoPos(servoPosX, servoPosY, true);
		}
		Sleep(2000);

		bool even = false;

		//visible and not moving
		bool visible = false;
		bool moving = true;
		//Starts loop until Escape key is pressed
		while (!visible || moving)
		{
			//original image
			Mat imgOriginal;
			bool bSuccess = cam.read(imgOriginal); // read a new frame from video

			if (!bSuccess) //if not success, break loop
			{
				cout << "Cannot read a frame from video stream" << endl;
				return -1;
			}
			//convert to grayscale
			Mat imgbGrayScale;
			cvtColor(imgOriginal, imgbGrayScale, COLOR_BGR2GRAY);
			//gets the thresholded image
			Mat imgThresholded;
			inRange(imgbGrayScale, Scalar(thresholdLevelL), Scalar(thresholdLevelH), imgThresholded);

			//morphological opening (removes small objects)
			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));
			//morphological opening (removes holes)
			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)));


			//Calculate the moments of the thresholded image
			Moments oMoments = moments(imgThresholded);

			double dM01 = oMoments.m01;
			double dM10 = oMoments.m10;
			double dArea = oMoments.m00;

			//
			if (dArea >= 100 && dArea < 2000)
			{
				visible = true;
				//calculate the position
				int x = (dM10 / dArea);
				int y = (dM01 / dArea);


				cout << "CALIBRATION: x: " << x << ", y: " << y << endl;
				cout << "CALIBRATION: servoPosX: " << servoPosX << ", servoPosY: " << servoPosY << endl;

				float floatX = (dM10 / dArea) / 640.0;
				float floatY = (dM01 / dArea) / 480.0;
				cout << "Laser beam pointing to X: " << x << ", Y: " << y << endl;


				//FANN feed data BEGIN
				float in[2] = { floatX, floatY };
				float servoPosXScaled = (servoPosX - 80) / 70.0;
				float servoPosYScaled = (servoPosY - 79) / 40.0;
				float out[2] = { servoPosXScaled, servoPosYScaled };

				if (even) {
					fann_train(ann, in, out);
					cout << "Training..." << endl;
				}
				else
				{
					calc_out = fann_run(ann, in);
					float diffX = fann_abs(calc_out[0] - servoPosXScaled);
					float diffY = fann_abs(calc_out[1] - servoPosYScaled);
					printf("Calibration test (%f, %f) -> (%f, %f), but should be (%f, %f) with a difference of (%f, %f)\n",
						floatX, floatY,
						calc_out[0], calc_out[1],
						servoPosXScaled, servoPosYScaled,
						diffX, diffY);
				}
				even = !even;


				//FANN feed data END

				// the first thing we do is pick a few training points from the sinc() function.

				sample_type m;
				m(0, 0) = floatX;
				m(1, 0) = floatY;
				samples.push_back(sample_type(m));
				targets.push_back((int)servoPosX * 10000 + (int)servoPosY);

				//draw yellow rectangle on original image
				cv::rectangle(imgOriginal,
					Point(x - 5, y - 5),
					Point(x + 5, y + 5),
					Scalar(80, 240, 255),
					2,
					8);

				if (previousX >= 0 && previousY >= 0 && x == previousX && y == previousY)
				{
					moving = false;
					float xCam = 1.0*x / 640.0;
					float yCam = 1.0*y / 480.0;
					controller.CalibrateCameraPos(xCam, yCam);
					cout << "Calibrated at X: " << x << ", Y: " << y << endl;
				}

				previousX = x;
				previousY = y;
			}
			else
			{
				visible = false;
			}
			//imshow("Camera View", imgOriginal);

			if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				//destroyWindow("Camera View");

				return 1;
			}
			if (!visible)
			{
				servoPosX = 80 + std::rand() % 70;
				servoPosY = 79 + std::rand() % 40;
				controller.MoveToServoPos(servoPosX, servoPosY, true);

				Sleep(500);
			}

		}
	}

	// Now setup a SVR trainer object.  It has three parameters, the kernel and

	// two parameters specific to SVR.

	svr_trainer<kernel_type> trainer;

	trainer.set_kernel(kernel_type(0.1));



	// This parameter is the usual regularization parameter.  It determines the trade-off

	// between trying to reduce the training error or allowing more errors but hopefully

	// improving the generalization of the resulting function.  Larger values encourage exact

	// fitting while smaller values of C may encourage better generalization.

	trainer.set_c(1000);



	// Epsilon-insensitive regression means we do regression but stop trying to fit a data

	// point once it is "close enough" to its target value.  This parameter is the value that

	// controls what we mean by "close enough".  In this case, I'm saying I'm happy if the

	// resulting regression function gets within 0.001 of the target value.

	trainer.set_epsilon_insensitivity(0.001);



	// Now do the training and save the results
	df = trainer.train(samples, targets);


	matrix<double, 2, 1> m;
	m(0, 0) = m(1, 0) = 25; cout << 25 * 10000 + 25 << "   " << df(m) << endl;

	m(0, 0) = m(1, 0) = 1; cout << 1 * 10000 + 1 << "   " << df(m) << endl;

	m(0, 0) = m(1, 0) = -4;  cout << -4 * 10000 + -4 << "   " << df(m) << endl;

	m(0, 0) = m(1, 0) = 5; cout << 5 * 10000 + 5 << "   " << df(m) << endl;



	cout << "Servo is calibrated by " << nbOfPoints << " points" << endl;
	return 0;
}