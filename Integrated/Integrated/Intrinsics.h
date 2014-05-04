#ifndef _INCLUDED_INTRINSICS_H
#define _INCLUDED_INTRINSICS_H

#include <cv.h>

class Intrinsics
{

public:
	
	// Constructor
	Intrinsics(cv::Mat cameraMatrix, cv::Mat distCoeffs);
	
	// Constructor for failures
	Intrinsics();

	// Public variables
	cv::Mat cm;			// cameraMatrix
	cv::Mat kc;			// distCoeffs
	double fc[2];
	double cc [2];
	double alpha_c;
	double err[2];

};

#endif