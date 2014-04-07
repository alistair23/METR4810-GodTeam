#ifndef _INCLUDED_TILE_H
#define _INCLUDED_TILE_H

#include <cv.h>
#include <highgui.h>

// A racetrack tile containing the image
// Origin for tile is defined at top left
class Tile {

public:

	Tile(cv::Mat img_in, bool lr_symmetric);

	// Compare this tile's binary (black/white) image to another 
	// binary image. Rotate this tile by rot first, 1 -> 90 degrees counter 
	// clockwise from east, etc. Returns % of matching pixels.
	double compare(cv::Mat& img_other, int rot);

private:

	std::vector<cv::Mat> imgs_;
	int total_pixels_;		// Number of pixels in image
};


#endif