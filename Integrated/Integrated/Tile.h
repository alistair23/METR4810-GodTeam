#ifndef _INCLUDED_TILE_H
#define _INCLUDED_TILE_H

#include <cv.h>
#include <highgui.h>

// A racetrack tile containing the image
// Origin for tile is defined at top left
// Entrance assumed to be from the right
class Tile {

public:

	// img_in is the source image
	// exit is 0 if exit is left, -1 if up, 1 if down
	Tile(cv::Mat img_in, int exit);

	// Compare this tile's binary (black/white) image to another 
	// binary image. Rotate this tile by rot first, 1 -> 90 degrees
	// counter clockwise, etc. Returns % of matching pixels.
	double compare(cv::Mat& img_other, int rot);

	int exit_;
	std::vector<cv::Mat> imgs_;
	int total_pixels_;		// Number of pixels in image
	
};


#endif