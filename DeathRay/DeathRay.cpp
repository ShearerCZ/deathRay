// DeathRay.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "LaserTracking.h"
#include "Phidget.h"
#include "ServoCtrl.h"
#include <dlib/svm.h>

using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
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

	LaserTracking laser(cap, ctrl, false);
	laser.Track();
	laser.Calibrate(ctrl, 10);
	//laser.ShootingRange(ctrl, false, false);
   /* laser.Calibrate(ctrl, 20);
	laser.ShootingRange(ctrl, false, false);*/
    //ctrl.MoveToServoPos(100, 100);
	//int result = laser.Track(true, true);
	cap.release();
	return 0;
}

//int _tmain(int argc, _TCHAR* argv[])
//{
//	PhidgetController pctrl;
//	ServoCtrl ctrl(pctrl, 2, 3);
//	return 0;
//}



// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt

/*

This is an example illustrating the use of the epsilon-insensitive support vector

regression object from the dlib C++ Library.


In this example we will draw some points from the sinc() function and do a

non-linear regression on them.

*/


//#include <iostream>
//
//#include <vector>
//
//
//#include <dlib/svm.h>
//
//
//using namespace std;
//
//using namespace dlib;
//
//
//double foo(double x, double y)
//
//{
//
//	x = x == 0 ? 1 : sin(x) / x;
//
//	y = y == 0 ? 1 : tan(y) / y;
//
//	return x * 1000 + y;
//
//}


//int main()
//
//{
//
//	// Here we declare that our samples will be 1 dimensional column vectors.
//
//	typedef matrix<double, 2, 1> sample_type;
//
//
//
//	// Now we are making a typedef for the kind of kernel we want to use.  I picked the
//
//	// radial basis kernel because it only has one parameter and generally gives good
//
//	// results without much fiddling.
//
//	typedef radial_basis_kernel<sample_type> kernel_type;
//
//
//
//
//
//	std::vector<sample_type> samples;
//
//	std::vector<double> targets;
//
//
//
//	// The first thing we do is pick a few training points from the sinc() function.
//
//	sample_type m;
//
//	for (double x = -10; x <= 4; x += 1)
//
//	{
//
//		m(0, 0) = x;
//
//		m(1, 0) = x;
//
//
//
//		samples.push_back(m);
//
//		targets.push_back(foo(x, x));
//
//	}
//
//
//
//	// Now setup a SVR trainer object.  It has three parameters, the kernel and
//
//	// two parameters specific to SVR.
//
//	svr_trainer<kernel_type> trainer;
//
//	trainer.set_kernel(kernel_type(0.1));
//
//
//
//	// This parameter is the usual regularization parameter.  It determines the trade-off
//
//	// between trying to reduce the training error or allowing more errors but hopefully
//
//	// improving the generalization of the resulting function.  Larger values encourage exact
//
//	// fitting while smaller values of C may encourage better generalization.
//
//	trainer.set_c(1000);
//
//
//
//	// Epsilon-insensitive regression means we do regression but stop trying to fit a data
//
//	// point once it is "close enough" to its target value.  This parameter is the value that
//
//	// controls what we mean by "close enough".  In this case, I'm saying I'm happy if the
//
//	// resulting regression function gets within 0.001 of the target value.
//
//	trainer.set_epsilon_insensitivity(0.001);
//
//
//
//	// Now do the training and save the results
//
//	decision_function<kernel_type> df = trainer.train(samples, targets);
//
//
//
//	// now we output the value of the sinc function for a few test points as well as the
//
//	// value predicted by SVR.
//
//	m(0, 0) = m(1, 0) = 2.5; cout << foo(2.5, 2.5) << "   " << df(m) << endl;
//
//	m(0, 0) = m(1, 0) = 0.1; cout << foo(0.1, 0.1) << "   " << df(m) << endl;
//
//	m(0, 0) = m(1, 0) = -4;  cout << foo(-4, -4) << "   " << df(m) << endl;
//
//	m(0, 0) = m(1, 0) = 5.0; cout << foo(5.0, 5.0) << "   " << df(m) << endl;
//
//
//
//	// The output is as follows:
//
//	//  0.239389   0.23905
//
//	//  0.998334   0.997331
//
//	// -0.189201   -0.187636
//
//	// -0.191785   -0.218924
//
//
//
//	// The first column is the true value of the sinc function and the second
//
//	// column is the output from the SVR estimate.
//
//
//
//	// We can also do 5-fold cross-validation and find the mean squared error and R-squared
//
//	// values.  Note that we need to randomly shuffle the samples first.  See the svm_ex.cpp
//
//	// for a discussion of why this is important.
//
//	randomize_samples(samples, targets);
//
//	cout << "MSE and R-Squared: " << cross_validate_regression_trainer(trainer, samples, targets, 5) << endl;
//
//	// The output is:
//
//	// MSE and R-Squared: 1.65984e-05    0.999901
//
//
//
//	return 0;
//
//}
