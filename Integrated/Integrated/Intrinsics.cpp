
#include "Intrinsics.h"

using namespace std;
using namespace cv;

/// Constructor
Intrinsics::Intrinsics(Mat cameraMatrix, Mat distCoeffs)
{

	// Extract vectors from cameraMatrix
	double fx = cameraMatrix.at<double>(0,0);
	double fy = cameraMatrix.at<double>(1,1);
	double cx = cameraMatrix.at<double>(0,2);
	double cy = cameraMatrix.at<double>(1,2);

	// Save to public variables
	cm = cameraMatrix;
	kc = distCoeffs;
	fc[0] = fx;
	fc[1] = fy;
	cc[0] = cx;
	cc[1] = cy;
	alpha_c = 0;	// No skew in OpenCV calibration I think...

}

// Empty constructor used for failures in CamCalib
Intrinsics::Intrinsics()
{
}