#include "Tile.h"
#include "Globals.h"

Tile::Tile(cv::Mat img_in, int exit):
	exit_(exit)
{

	// Resize to our scale
	double scale = SOURCE_M_PER_PIX / M_PER_PIX;
	cv::resize(img_in, img_in, cv::Size(0,0), scale, scale);

	// Color threshold to get binary image
	inRange(img_in, cv::Scalar(25, 25, 25),
		cv::Scalar(255, 255, 255), img_in);

	// Store the possible rotations 
	imgs_.push_back(img_in);
	for (int i = 1; i < 4; i++) {
		cv::Mat dst;
		cv::Point2f centre(img_in.cols/2., img_in.rows/2.); 
		cv::Mat r = cv::getRotationMatrix2D(centre, 90 * i, 1.0);
		cv::warpAffine(img_in, dst, r, cv::Size(img_in.cols, img_in.rows));
		imgs_.push_back(dst);
	}

	// Count number of pixels
	total_pixels_ = img_in.rows * img_in.cols; 
}

// Compare this tile's binary (black/white) image to another 
// binary image. Returns % of matching pixels.
double Tile::compare(cv::Mat& img_other, int rot ) {

	cv::Mat temp;
	cv::compare(imgs_[rot], img_other, temp, cv::CMP_EQ);
	int similar_pixels  = cv::countNonZero(temp);
	return similar_pixels /  (double) total_pixels_;
}