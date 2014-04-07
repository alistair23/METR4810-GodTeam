#ifndef _INCLUDED_VISION_H
#define _INCLUDED_VISION_H

#include <cv.h>
#include <highgui.h>

#include "Racetrack.h"
#include "Tile.h"

class Vision {

public:

	Vision();

	Racetrack extractRacetrack(cv::Mat& img_in);
	
private:

	// Returns euclidean distance between two opencv points
	double dist(cv::Point p1, cv::Point p2);

	// Given a image of the racetrack, perspective transforms it to get
	// top down view aligned with circle markers and scaled to 1mm:1pixel
	// Centre of marker pair is provided in pair_centre.
	// Returns true if success in finding marker pair
	bool transformTrackImage(cv::Mat& img_in, cv::Point& pair_centre);

	// Return vertices of a rotated rectangle in clockwise order,
	// starting from top left
	std::vector<cv::Point2f> sortVertices(cv::RotatedRect& rect);

	cv::Point2f findBoundary(cv::Mat& img, cv::Point2f start, float dir); 

	Tile finish_tile_1;
	Tile finish_tile_2;
};


#endif
