#ifndef _INCLUDED_VISION_H
#define _INCLUDED_VISION_H

#include <cv.h>
#include <highgui.h>
#include <vector>

#include "Point.h"
#include "Tile.h"

class Vision {

public:

	Vision();

	// For finding the perspective transform for a camera, which when
	// applied to image will give unwarped top-down view.
	// Input image must contain our special marker used during setup time
	// perspective transform matrix is output through transform_out
	void getTransform(cv::Mat& img_in, cv::Mat& transform_output);

	// Given an input image (BGR), looks for the car markers 
	// (concentric circles). My car's centre is output in my_car_p1,
	// and off-centre circle location in my_car_p2. Other car markers
	// output in other_cars
	void getCarMarkers(cv::Mat& img_in, cv::Point2f& my_car_p1,
		cv::Point2f& my_car_p2, std::vector<cv::Point2f>& other_cars);

	// WORK IN PROGRESS
	std::vector<Point> extractRacetrack(cv::Mat& img_thresh, cv::Point2f origin,
		cv::Point2f start_position, float start_orientation, cv::Point2f end_position); 

	bool transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre);	// PROBABLY WILL DELETE


private:

	// Returns euclidean distance between two opencv points
	float dist(cv::Point2f& p1, cv::Point2f& p2);

	// Returns euclidean distance squared between two opencv points
	float dist_sq(cv::Point2f& p1, cv::Point2f& p2);
	
	// Returns atan2(p2.y - p1.y, p2.x - p1.x)
	float angle(cv::Point2f& p1, cv::Point2f& p2);

	// Return vertices of a rotated rectangle in clockwise order,
	// starting from top left
	std::vector<cv::Point2f> sortVertices(cv::RotatedRect& rect);

	// Get average colour around point in image
	cv::Scalar getColour(cv::Mat& img, cv::Point2f p, int pix_length = 5);

	cv::Point2f getBoundary(cv::Mat& img, cv::Point2f start, float dir);

	int getTileType(cv::Mat& img_roi, int rot);

	Tile finish_tile_1;
	Tile finish_tile_2;

	std::vector<Tile> tiles;
};


#endif
