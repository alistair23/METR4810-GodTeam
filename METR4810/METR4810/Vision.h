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

	void getTransform(cv::Mat& img_in, cv::Mat& transform_output);
	bool transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre);

	std::vector<Point> extractRacetrack(cv::Mat& img_thresh, cv::Point2f origin, cv::Point2f start_position, float start_orientation, cv::Point2f end_position); 

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
