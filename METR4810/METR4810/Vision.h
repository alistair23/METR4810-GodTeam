#ifndef _INCLUDED_VISION_H
#define _INCLUDED_VISION_H

#include <cv.h>
#include <highgui.h>

#include "Racetrack.h"
#include "Tile.h"

class Vision {

public:

	Vision();

	bool transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre);

	Racetrack extractRacetrack(cv::Mat& img_thresh, cv::Point2f origin, cv::Point2f start_position, float start_orientation, cv::Point2f end_position); 
	
private:

	// Returns euclidean distance between two opencv points
	float dist(cv::Point2f& p1, cv::Point2f& p2);
	
	// Returns atan2(p2.y - p1.y, p2.x - p1.x)
	float angle(cv::Point2f& p1, cv::Point2f& p2);

	// Return vertices of a rotated rectangle in clockwise order,
	// starting from top left
	std::vector<cv::Point2f> sortVertices(cv::RotatedRect& rect);

	cv::Point2f findBoundary(cv::Mat& img, cv::Point2f start, float dir);
	float findImgGradient(cv::Mat& img, cv::Point2f p);

	int getTileType(cv::Mat& img_roi, int rot);

	Tile finish_tile_1;
	Tile finish_tile_2;

	std::vector<Tile> tiles;
};


#endif
